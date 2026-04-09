// SPDX-License-Identifier: GPL-2.0-only
/*
 * TCP CUBIC: Binary Increase Congestion control for TCP v2.3
 * Home page:
 *      http://netsrv.csc.ncsu.edu/twiki/bin/view/Main/BIC
 * This is from the implementation of CUBIC TCP in
 * Sangtae Ha, Injong Rhee and Lisong Xu,
 *  "CUBIC: A New TCP-Friendly High-Speed TCP Variant"
 *  in ACM SIGOPS Operating System Review, July 2008.
 * Available from:
 *  http://netsrv.csc.ncsu.edu/export/cubic_a_new_tcp_2008.pdf
 *
 * CUBIC integrates a new slow start algorithm, called HyStart.
 * The details of HyStart are presented in
 *  Sangtae Ha and Injong Rhee,
 *  "Taming the Elephants: New TCP Slow Start", NCSU TechReport 2008.
 * Available from:
 *  http://netsrv.csc.ncsu.edu/export/hystart_techreport_2008.pdf
 *
 * All testing results are available from:
 * http://netsrv.csc.ncsu.edu/wiki/index.php/TCP_Testing
 *
 * Unless CUBIC is enabled and congestion window is large
 * this behaves the same as the original Reno.
 */

#include <linux/mm.h>
#include <linux/btf.h>
#include <linux/btf_ids.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <net/tcp.h>

#define BICTCP_BETA_SCALE    1024	/* Scale factor beta calculation
					 * max_cwnd = snd_cwnd * beta
					 */
#define	BICTCP_HZ		10	/* BIC HZ 2^10 = 1024 */

/* Two methods of hybrid slow start */
#define HYSTART_ACK_TRAIN	0x1
#define HYSTART_DELAY		0x2

/* Number of delay samples for detecting the increase of delay */
#define HYSTART_MIN_SAMPLES	8
#define HYSTART_DELAY_MIN	(4000U)	/* 4 ms */
#define HYSTART_DELAY_MAX	(16000U)	/* 16 ms */
#define HYSTART_DELAY_THRESH(x)	clamp(x, HYSTART_DELAY_MIN, HYSTART_DELAY_MAX)

static int fast_convergence __read_mostly = 1;
static int beta __read_mostly = 717;	/* = 717/1024 (BICTCP_BETA_SCALE) */
static int initial_ssthresh __read_mostly;
static int bic_scale __read_mostly = 41;
static int tcp_friendliness __read_mostly = 1;

static int hystart_detect __read_mostly = HYSTART_ACK_TRAIN | HYSTART_DELAY;
static int hystart_low_window __read_mostly = 16;
static int hystart_ack_delta_us __read_mostly = 2000;

static u32 cube_rtt_scale __read_mostly;
static u32 beta_scale __read_mostly;
static u64 cube_factor __read_mostly;

/* Note parameters that are used for precomputing scale factors are read-only */
module_param(fast_convergence, int, 0644);
MODULE_PARM_DESC(fast_convergence, "turn on/off fast convergence");
module_param(beta, int, 0644);
MODULE_PARM_DESC(beta, "beta for multiplicative increase");
module_param(initial_ssthresh, int, 0644);
MODULE_PARM_DESC(initial_ssthresh, "initial value of slow start threshold");
module_param(bic_scale, int, 0444);
MODULE_PARM_DESC(bic_scale, "scale (scaled by 1024) value for bic function (bic_scale/1024)");
module_param(tcp_friendliness, int, 0644);
MODULE_PARM_DESC(tcp_friendliness, "turn on/off tcp friendliness");
module_param(hystart_detect, int, 0644);
MODULE_PARM_DESC(hystart_detect, "hybrid slow start detection mechanisms"
		 " 1: packet-train 2: delay 3: both packet-train and delay");
module_param(hystart_low_window, int, 0644);
MODULE_PARM_DESC(hystart_low_window, "lower bound cwnd for hybrid slow start");
module_param(hystart_ack_delta_us, int, 0644);
MODULE_PARM_DESC(hystart_ack_delta_us, "spacing between ack's indicating train (usecs)");
//////////////////////// SEARCH ////////////////////////
/*	enable SEARCH with command:
 		sudo sh -c "echo '1' > /sys/module/your_module_name/parameters/slow_start_mode"
	enable HyStart with command:
 		sudo sh -c "echo '2' > /sys/module/cubic_with_search/parameters/slow_start_mode"  
	disable both SEARCH and HyStart with command:
 		sudo sh -c "echo '0' > /sys/module/cubic_with_search/parameters/slow_start_mode" 
*/

#define MAX_US_INT 0xffff 
#define SEARCH_BINS 10												/* Number of bins in a window */
#define SEARCH_EXTRA_ACKED_BINS 1  									/* Number of additional bins to calculate delivery window (as this is cumulative, we need one more bin) */
#define SEARCH_EXTRA_SENT_BINS 40									/* Number of additional bins to cover data after shiftting by RTT */
#define SEARCH_ACKED_BINS (SEARCH_BINS + SEARCH_EXTRA_ACKED_BINS)	/* Number of total bins in a acked window */
#define SEARCH_SENT_BINS (SEARCH_BINS + SEARCH_EXTRA_SENT_BINS)		/* Number of total bins in a sent window */
#define SEARCH_VERSION 40 /* Jut for logging */
#define SEARCH_DRAIN_ACKEDSEG_THRESH 16        		/* ACKed-segment threshold to permit CWND increase during drain */


/* Define an enum for the slow start mode */
enum {
    SS_LEGACY = 0, /* No slow start algorithm is used */
    SS_SEARCH = 1, /* Enable the SEARCH slow start algorithm */
    SS_HYSTART = 2 /* Enable the HyStart slow start algorithm */
};

//new_change: app_limited
enum unset_bin_duration {
    RESET_BIN_DURATION_TRUE,  // Reset bin duration
    RESET_BIN_DURATION_FALSE    // Do not reset bin duration
}; //

/* Set the default mode */
static int slow_start_mode __read_mostly = SS_SEARCH;
static int search_window_duration_factor __read_mostly = 35;
static int search_thresh __read_mostly = 35;
static int search_alpha = MAX_US_INT;    // 2
static int debug_port __read_mostly = 5201;

// Module parameters used by SEARCH
module_param(slow_start_mode, int, 0644);
MODULE_PARM_DESC(slow_start_mode, "0: No Algorithm, 1: SEARCH, 2: HyStart");
module_param(search_window_duration_factor, int, 0644);
MODULE_PARM_DESC(search_window_duration_factor, "Multiply with (initial RTT / 10) to set the window size");
module_param(search_thresh, int, 0644);
MODULE_PARM_DESC(search_thresh, "Threshold for exiting from slow start in percentage");
module_param(debug_port, int, 0644);
MODULE_PARM_DESC(debug_port, "Minimum threshold of missed bins before resetting SEARCH");
module_param(search_alpha, int, 0644);
MODULE_PARM_DESC(search_alpha, "Alpha factor for determining missed bin limit in SEARCH. Defaults to 2, representing two RTTs.");



/* BIC TCP Parameters */
struct bictcp {
	u32	cnt;		/* increase cwnd by 1 after ACKs */
	u32	last_max_cwnd;	/* last maximum snd_cwnd */
	u32	last_cwnd;	/* the last snd_cwnd */
	u32	last_time;	/* time when updated last_cwnd */
	u32	bic_origin_point;/* origin point of bic function */
	u32	bic_K;		/* time to origin point
				   from the beginning of the current epoch */
	u32	delay_min;	/* min delay (usec) */
	u32	epoch_start;	/* beginning of an epoch */
	u32	ack_cnt;	/* number of acks */
	u32	tcp_cwnd;	/* estimated tcp cwnd */

	/* Union of HyStart and SEARCH variables */
	union {
		/* HyStart variables */
		struct {
			u16	unused;
			u8	sample_cnt;/* number of samples to decide curr_rtt */
			u8	found;		/* the exit point is found? */
			u32	round_start;	/* beginning of each round */
			u32	end_seq;	/* end_seq of the round */
			u32	last_ack;	/* last time when the ACK spacing is close */
			u32	curr_rtt;	/* the minimum rtt of current round */
		}hystart;

		/* SEARCH variables */
		struct {
			u32	bin_duration_us;	/* duration of each bin in microsecond */
			s32	curr_idx;	/* total number of bins */
			u32	bin_end_us;	/* end time of the latest bin in microsecond */
			u16	acked_bin[SEARCH_ACKED_BINS];	/* array to keep acked bytes for bins */
			u16 sent_bin[SEARCH_SENT_BINS];		/* array to keep sent bytes for bins */
			u8	scale_factor;	/* scale factor to fit the value with bin size*/
			u32 search_targeted_cwnd;  					/* Rollback CWND target = BDP estimate from one RTT earlier; used to initiate SEARCH drain */
			u8 search_cwnd_reduction_to_target; 			/* Triggers CWND drain toward search_targeted_cwnd */
			u32 search_drain_ackedseg;      	 			/* Accumulates number of segments ACKed during SEARCH drain */
			u32 prior_delivered;			/* Number of packets delivered previously*/
		}search;
	};
};

static inline void bictcp_search_reset(struct sock *sk, enum unset_bin_duration flag)
{
	struct bictcp *ca = inet_csk_ca(sk);
	struct tcp_sock *tp = tcp_sk(sk);

	memset(ca->search.sent_bin, 0, sizeof(ca->search.sent_bin));
	memset(ca->search.acked_bin, 0, sizeof(ca->search.acked_bin));
	ca->search.curr_idx = -1;
	ca->search.bin_end_us = 0;
	ca->search.scale_factor = 0;
	ca->search.search_targeted_cwnd = 0;
	ca->search.search_cwnd_reduction_to_target = 0;
	ca->search.search_drain_ackedseg = 0;
	ca->search.prior_delivered = tp->delivered;
	if (flag == RESET_BIN_DURATION_TRUE) 
		ca->search.bin_duration_us = 0; 
}

static inline void bictcp_reset(struct bictcp *ca)
{
	memset(ca, 0, offsetof(struct bictcp, hystart.unused));
	if (slow_start_mode == SS_HYSTART)
		ca->hystart.found = 0;
}

static inline u32 bictcp_clock_us(const struct sock *sk)
{
	return tcp_sk(sk)->tcp_mstamp;
}

static inline void bictcp_hystart_reset(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);

	ca->hystart.round_start = ca->hystart.last_ack = bictcp_clock_us(sk);
	ca->hystart.end_seq = tp->snd_nxt;
	ca->hystart.curr_rtt = ~0U;
	ca->hystart.sample_cnt = 0;
}

static inline void search_print_header(struct bictcp *ca)
{
	printk(KERN_INFO "[CCRG]: [flow pointer: 0x%p] ",
		ca);
}

__bpf_kfunc static void cubictcp_init(struct sock *sk)
{
	struct bictcp *ca = inet_csk_ca(sk);

	bictcp_reset(ca);

	if (slow_start_mode == SS_SEARCH)
		bictcp_search_reset(sk, RESET_BIN_DURATION_TRUE);

	if (slow_start_mode == SS_HYSTART)
		bictcp_hystart_reset(sk);

	if (slow_start_mode != SS_HYSTART && initial_ssthresh)
		tcp_sk(sk)->snd_ssthresh = initial_ssthresh;
}

__bpf_kfunc static void cubictcp_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{

	if (event == CA_EVENT_TX_START) {
		struct bictcp *ca = inet_csk_ca(sk);
		u32 now = tcp_jiffies32;
		s32 delta;

		delta = now - tcp_sk(sk)->lsndtime;

		/* We were application limited (idle) for a while.
		 * Shift epoch_start to keep cwnd growth to cubic curve.
		 */
		if (ca->epoch_start && delta > 0) {
			ca->epoch_start += delta;
			if (after(ca->epoch_start, now))
				ca->epoch_start = now;
		}
		return;
	}

	if (event == CA_EVENT_CWND_RESTART) {
		if (slow_start_mode == SS_SEARCH)
			bictcp_search_reset(sk, RESET_BIN_DURATION_TRUE);
		return;
	}
	return;
}

/* calculate the cubic root of x using a table lookup followed by one
 * Newton-Raphson iteration.
 * Avg err ~= 0.195%
 */
static u32 cubic_root(u64 a)
{
	u32 x, b, shift;
	/*
	 * cbrt(x) MSB values for x MSB values in [0..63].
	 * Precomputed then refined by hand - Willy Tarreau
	 *
	 * For x in [0..63],
	 *   v = cbrt(x << 18) - 1
	 *   cbrt(x) = (v[x] + 10) >> 6
	 */
	static const u8 v[] = {
		/* 0x00 */    0,   54,   54,   54,  118,  118,  118,  118,
		/* 0x08 */  123,  129,  134,  138,  143,  147,  151,  156,
		/* 0x10 */  157,  161,  164,  168,  170,  173,  176,  179,
		/* 0x18 */  181,  185,  187,  190,  192,  194,  197,  199,
		/* 0x20 */  200,  202,  204,  206,  209,  211,  213,  215,
		/* 0x28 */  217,  219,  221,  222,  224,  225,  227,  229,
		/* 0x30 */  231,  232,  234,  236,  237,  239,  240,  242,
		/* 0x38 */  244,  245,  246,  248,  250,  251,  252,  254,
	};

	b = fls64(a);
	if (b < 7) {
		/* a in [0..63] */
		return ((u32)v[(u32)a] + 35) >> 6;
	}

	b = ((b * 84) >> 8) - 1;
	shift = (a >> (b * 3));

	x = ((u32)(((u32)v[shift] + 10) << b)) >> 6;

	/*
	 * Newton-Raphson iteration
	 *                         2
	 * x    = ( 2 * x  +  a / x  ) / 3
	 *  k+1          k         k
	 */
	x = (2 * x + (u32)div64_u64(a, (u64)x * (u64)(x - 1)));
	x = ((x * 341) >> 10);
	return x;
}

/*
 * Compute congestion window to use.
 */
static inline void bictcp_update(struct bictcp *ca, u32 cwnd, u32 acked)
{
	u32 delta, bic_target, max_cnt;
	u64 offs, t;

	ca->ack_cnt += acked;	/* count the number of ACKed packets */

	if (ca->last_cwnd == cwnd &&
	    (s32)(tcp_jiffies32 - ca->last_time) <= HZ / 32)
		return;

	/* The CUBIC function can update ca->cnt at most once per jiffy.
	 * On all cwnd reduction events, ca->epoch_start is set to 0,
	 * which will force a recalculation of ca->cnt.
	 */
	if (ca->epoch_start && tcp_jiffies32 == ca->last_time)
		goto tcp_friendliness;

	ca->last_cwnd = cwnd;
	ca->last_time = tcp_jiffies32;

	if (ca->epoch_start == 0) {
		ca->epoch_start = tcp_jiffies32;	/* record beginning */
		ca->ack_cnt = acked;			/* start counting */
		ca->tcp_cwnd = cwnd;			/* syn with cubic */

		if (ca->last_max_cwnd <= cwnd) {
			ca->bic_K = 0;
			ca->bic_origin_point = cwnd;
		} else {
			/* Compute new K based on
			 * (wmax-cwnd) * (srtt>>3 / HZ) / c * 2^(3*bictcp_HZ)
			 */
			ca->bic_K = cubic_root(cube_factor
					       * (ca->last_max_cwnd - cwnd));
			ca->bic_origin_point = ca->last_max_cwnd;
		}
	}

	/* cubic function - calc*/
	/* calculate c * time^3 / rtt,
	 *  while considering overflow in calculation of time^3
	 * (so time^3 is done by using 64 bit)
	 * and without the support of division of 64bit numbers
	 * (so all divisions are done by using 32 bit)
	 *  also NOTE the unit of those veriables
	 *	  time  = (t - K) / 2^bictcp_HZ
	 *	  c = bic_scale >> 10
	 * rtt  = (srtt >> 3) / HZ
	 * !!! The following code does not have overflow problems,
	 * if the cwnd < 1 million packets !!!
	 */

	t = (s32)(tcp_jiffies32 - ca->epoch_start);
	t += usecs_to_jiffies(ca->delay_min);
	/* change the unit from HZ to bictcp_HZ */
	t <<= BICTCP_HZ;
	do_div(t, HZ);

	if (t < ca->bic_K)		/* t - K */
		offs = ca->bic_K - t;
	else
		offs = t - ca->bic_K;

	/* c/rtt * (t-K)^3 */
	delta = (cube_rtt_scale * offs * offs * offs) >> (10+3*BICTCP_HZ);
	if (t < ca->bic_K)                            /* below origin*/
		bic_target = ca->bic_origin_point - delta;
	else                                          /* above origin*/
		bic_target = ca->bic_origin_point + delta;

	/* cubic function - calc bictcp_cnt*/
	if (bic_target > cwnd) {
		ca->cnt = cwnd / (bic_target - cwnd);
	} else {
		ca->cnt = 100 * cwnd;              /* very small increment*/
	}

	/*
	 * The initial growth of cubic function may be too conservative
	 * when the available bandwidth is still unknown.
	 */
	if (ca->last_max_cwnd == 0 && ca->cnt > 20)
		ca->cnt = 20;	/* increase cwnd 5% per RTT */

tcp_friendliness:
	/* TCP Friendly */
	if (tcp_friendliness) {
		u32 scale = beta_scale;

		delta = (cwnd * scale) >> 3;
		while (ca->ack_cnt > delta) {		/* update tcp cwnd */
			ca->ack_cnt -= delta;
			ca->tcp_cwnd++;
		}

		if (ca->tcp_cwnd > cwnd) {	/* if bic is slower than tcp */
			delta = ca->tcp_cwnd - cwnd;
			max_cnt = cwnd / delta;
			if (ca->cnt > max_cnt)
				ca->cnt = max_cnt;
		}
	}

	/* The maximum rate of cwnd increase CUBIC allows is 1 packet per
	 * 2 packets ACKed, meaning cwnd grows at 1.5x per RTT.
	 */
	ca->cnt = max(ca->cnt, 2U);
}

__bpf_kfunc static void cubictcp_cong_avoid(struct sock *sk, u32 ack, u32 acked)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);

	if (!tcp_is_cwnd_limited(sk))
		return;

	// if SEARCH is in drain, set the cwnd based on drain rate
	if (tcp_in_slow_start(tp) && ca->search.search_cwnd_reduction_to_target == 1)
		return;

	if (tcp_in_slow_start(tp)) {
		acked = tcp_slow_start(tp, acked);
		if (!acked)
			return;
	}
	bictcp_update(ca, tcp_snd_cwnd(tp), acked);
	tcp_cong_avoid_ai(tp, ca->cnt, acked);
}

__bpf_kfunc static u32 cubictcp_recalc_ssthresh(struct sock *sk)
{
	const struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);

	ca->epoch_start = 0;	/* end of epoch */

	/* Wmax and fast convergence */
	if (tcp_snd_cwnd(tp) < ca->last_max_cwnd && fast_convergence)
		ca->last_max_cwnd = (tcp_snd_cwnd(tp) * (BICTCP_BETA_SCALE + beta))
			/ (2 * BICTCP_BETA_SCALE);
	else
		ca->last_max_cwnd = tcp_snd_cwnd(tp);

	return max((tcp_snd_cwnd(tp) * beta) / BICTCP_BETA_SCALE, 2U);
}

__bpf_kfunc static void cubictcp_state(struct sock *sk, u8 new_state)
{
	if (new_state == TCP_CA_Loss) {
		bictcp_reset(inet_csk_ca(sk));

		if (slow_start_mode == SS_SEARCH)
			bictcp_search_reset(sk, RESET_BIN_DURATION_TRUE); //new_change: app_limited

		if (slow_start_mode == SS_HYSTART)
			bictcp_hystart_reset(sk);
	}
}


/* Account for TSO/GRO delays.
 * Otherwise short RTT flows could get too small ssthresh, since during
 * slow start we begin with small TSO packets and ca->delay_min would
 * not account for long aggregation delay when TSO packets get bigger.
 * Ideally even with a very small RTT we would like to have at least one
 * TSO packet being sent and received by GRO, and another one in qdisc layer.
 * We apply another 100% factor because @rate is doubled at this point.
 * We cap the cushion to 1ms.
 */
static u32 hystart_ack_delay(const struct sock *sk)
{
	unsigned long rate;

	rate = READ_ONCE(sk->sk_pacing_rate);
	if (!rate)
		return 0;
	return min_t(u64, USEC_PER_MSEC,
		     div64_ul((u64)sk->sk_gso_max_size * 4 * USEC_PER_SEC, rate));
}

static void hystart_update(struct sock *sk, u32 delay)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);
	u32 threshold;

	if (after(tp->snd_una, ca->hystart.end_seq))
		bictcp_hystart_reset(sk);

	/* hystart triggers when cwnd is larger than some threshold */
	if (tcp_snd_cwnd(tp) < hystart_low_window)
		return;

	if (hystart_detect & HYSTART_ACK_TRAIN) {
		u32 now = bictcp_clock_us(sk);

		/* first detection parameter - ack-train detection */
		if ((s32)(now - ca->hystart.last_ack) <= hystart_ack_delta_us) {
			ca->hystart.last_ack = now;

			threshold = ca->delay_min + hystart_ack_delay(sk);

			/* Hystart ack train triggers if we get ack past
			 * ca->delay_min/2.
			 * Pacing might have delayed packets up to RTT/2
			 * during slow start.
			 */
			if (sk->sk_pacing_status == SK_PACING_NONE)
				threshold >>= 1;

			if ((s32)(now - ca->hystart.round_start) > threshold) {
				ca->hystart.found = 1;
				pr_debug("hystart_ack_train (%u > %u) delay_min %u (+ ack_delay %u) cwnd %u\n",
					 now - ca->hystart.round_start, threshold,
					 ca->delay_min, hystart_ack_delay(sk), tcp_snd_cwnd(tp));
				NET_INC_STATS(sock_net(sk),
					      LINUX_MIB_TCPHYSTARTTRAINDETECT);
				NET_ADD_STATS(sock_net(sk),
					      LINUX_MIB_TCPHYSTARTTRAINCWND,
					      tcp_snd_cwnd(tp));
				tp->snd_ssthresh = tcp_snd_cwnd(tp);

				search_print_header(ca);
				printk(KERN_CONT "HyStart_INFO: [now %u] [exit condition was met [cwnd %u] [ssthresh %u] [curr_rtt %u] [delay_min %u]\n", bictcp_clock_us(sk), tcp_snd_cwnd(tp), tp->snd_ssthresh, ca->hystart.curr_rtt, ca->delay_min);				

			}
		}
	}

	if (hystart_detect & HYSTART_DELAY) {
		/* obtain the minimum delay of more than sampling packets */
		if (ca->hystart.curr_rtt > delay)
			ca->hystart.curr_rtt = delay;
		if (ca->hystart.sample_cnt < HYSTART_MIN_SAMPLES) {
			ca->hystart.sample_cnt++;
		} else {
			if (ca->hystart.curr_rtt > ca->delay_min +
			    HYSTART_DELAY_THRESH(ca->delay_min >> 3)) {
				ca->hystart.found = 1;
				NET_INC_STATS(sock_net(sk),
					      LINUX_MIB_TCPHYSTARTDELAYDETECT);
				NET_ADD_STATS(sock_net(sk),
					      LINUX_MIB_TCPHYSTARTDELAYCWND,
					      tcp_snd_cwnd(tp));
				tp->snd_ssthresh = tcp_snd_cwnd(tp);

				search_print_header(ca);
				printk(KERN_CONT "HyStart_INFO: [now %u] [exit condition was met [cwnd %u] [ssthresh %u] [curr_rtt %u] [delay_min %u]\n", bictcp_clock_us(sk), tcp_snd_cwnd(tp), tp->snd_ssthresh, ca->hystart.curr_rtt, ca->delay_min);				

			}
		}
	}
}

//////////////////////// SEARCH ////////////////////////


/* Scale bin value to fit bin size, rescale previous bins.
 * Return amount scaled.
 */
static inline u8 search_bit_shifting(struct sock *sk, u64 bin_value) 
{

	struct bictcp *ca = inet_csk_ca(sk);
	u8 num_shift = 0; 
	u32 i = 0;


	/* Adjust bin_value if it's greater than MAX_BIN_VALUE */
	while (bin_value > MAX_US_INT) {
		num_shift += 1;
		bin_value >>= 1;  /* divide bin_value by 2 */
	}

	if (num_shift == 0)
    	return 0;

	/* Adjust all previous acked and sent bins according to the new num_shift */
	for (i = 0; i < SEARCH_ACKED_BINS; i++) 
		ca->search.acked_bin[i] >>= num_shift;

	for (i = 0; i < SEARCH_SENT_BINS; i++)
		ca->search.sent_bin[i] >>= num_shift;

	/* Update the scale factor */
	ca->search.scale_factor += num_shift;

	return num_shift;
}

/*
 * SEARCH: Initialize measurement bins.
 *
 * Called on first ACK reception to establish the SEARCH window structure.
 * Sets bin duration, end timestamp, and populates the first bin with
 * cumulative acked and sent bytes (scaled if necessary).
 */
static void search_init_bins(struct sock *sk, u32 now_us, u32 rtt_us)
{
	struct bictcp *ca = inet_csk_ca(sk);
	struct tcp_sock *tp = tcp_sk(sk);
	u8 amount_scaled = 0; 
	u64 acked_value = 0;
	u64 sent_value = 0;
	u64 largest_val = 0;

	if (ca->search.bin_duration_us == 0) //new_change: app_limited
		ca->search.bin_duration_us = (rtt_us * search_window_duration_factor) / (SEARCH_BINS * 10);

	ca->search.bin_end_us = now_us + ca->search.bin_duration_us;
	ca->search.curr_idx = 0;


	acked_value = tp->bytes_acked;
	sent_value = tp->bytes_sent;

	/* 
	 * Prevent bin overflow by right-shifting both acked and sent values
	 * proportionally if either exceeds MAX_US_INT. This ensures consistent scaling
	 * across arrays.
	 */
	if (acked_value > MAX_US_INT || sent_value > MAX_US_INT) {
		largest_val = (acked_value > sent_value) ? acked_value : sent_value;
		amount_scaled = search_bit_shifting(sk, largest_val);
		acked_value >>= amount_scaled;
		sent_value  >>= amount_scaled;
	}

	ca->search.acked_bin[0] = acked_value;
	ca->search.sent_bin[0] = sent_value;
}

/*
 * SEARCH: Advance bin windows and maintain temporal continuity.
 *
 * Updates bin arrays as time progresses:
 *  - Computes the number of passed bins since the last update.
 *  - Resets bins if too much time has elapsed (missed bins).
 *  - Filled intermediate bins with last known value when multiple bins have passed.
 *  - Applies dynamic scaling to prevent overflow.
 */

static void search_update_bins(struct sock *sk, u32 now_us, u32 rtt_us)
{
	struct bictcp *ca = inet_csk_ca(sk);
	struct tcp_sock *tp = tcp_sk(sk);
	u32 passed_bins = 0;
	u32 i = 0;
	u64 acked_value = 0;
	u64 sent_value = 0;
	u8 amount_scaled = 0; 
	u32 initial_rtt = 0; 
	u64 largest_val = 0;


	/* If passed_bins greater than 1, it means we have some missed bins */
	passed_bins = ((now_us - ca->search.bin_end_us) / ca->search.bin_duration_us) + 1;

	/*
	 * If the RTT / bin duration is greater than the number of missed
	 * bins, that means it has been at least one RTT since the last bin
	 * was filled.  In this case, computing the delivered bytes over an
	 * RTT is unreliable so SEARCH should be reset.
	 *
	 * When this condition is met:
	 *   - If passed_bins exceeds SEARCH_BINS, perform a complete SEARCH reset including unseting bin_duration. The bin_duration will be reset upon receiving the next ack..
	 *   - Otherwise, perform a partial SEARCH reset that preserves the existing bin duration.
	 *
	 * After resetting the SEARCH state, reinitialize the bins using the current timestamp
	 * and RTT.
	 *
	 */
	initial_rtt = ca->search.bin_duration_us * SEARCH_BINS * 10 / search_window_duration_factor;

	if (passed_bins > search_alpha * (initial_rtt / ca->search.bin_duration_us)) {


		if (passed_bins > SEARCH_BINS){
			bictcp_search_reset(sk, RESET_BIN_DURATION_TRUE);
		} else {
			bictcp_search_reset(sk, RESET_BIN_DURATION_FALSE);
		}
	    	search_init_bins(sk, now_us, rtt_us);
	    	return;
	}


	for (i = ca->search.curr_idx + 1; i < ca->search.curr_idx + passed_bins; i++){

		ca->search.acked_bin[i % SEARCH_ACKED_BINS] = ca->search.acked_bin[ca->search.curr_idx % SEARCH_ACKED_BINS];
		ca->search.sent_bin[i % SEARCH_SENT_BINS] = ca->search.sent_bin[ca->search.curr_idx % SEARCH_SENT_BINS];
	}

	ca->search.bin_end_us += passed_bins * ca->search.bin_duration_us;
	ca->search.curr_idx += passed_bins;

	/* Calculate bin_value by dividing bytes_acked and bytes_sent by 2^scale_factor */
	acked_value = tp->bytes_acked >> ca->search.scale_factor; 
	sent_value = tp->bytes_sent >> ca->search.scale_factor;

	if (acked_value > MAX_US_INT || sent_value > MAX_US_INT) {
		largest_val = (acked_value > sent_value) ? acked_value : sent_value;
		amount_scaled = search_bit_shifting(sk, largest_val);
		acked_value >>= amount_scaled;
		sent_value  >>= amount_scaled;
	}

	/* Assign the bin_value to the current bin */
	ca->search.acked_bin[ca->search.curr_idx % SEARCH_ACKED_BINS] = acked_value;
	ca->search.sent_bin[ca->search.curr_idx % SEARCH_SENT_BINS] = sent_value;
}

/* Calculate delivered bytes for a window*/
static inline u64 search_compute_delivered_window(struct sock *sk, s32 left, s32 right)
{
	struct bictcp *ca = inet_csk_ca(sk);
	u64 delivered = 0;

	delivered = ca->search.acked_bin[right % SEARCH_ACKED_BINS] - ca->search.acked_bin[left % SEARCH_ACKED_BINS];

	return delivered;
}

/* Calculate sent bytes for a window considering interpolation */
static inline u64 search_compute_sent_window(struct sock *sk, s32 left, s32 right, u32 fraction)
{
	struct bictcp *ca = inet_csk_ca(sk);
	u64 sent = 0;

	sent = ca->search.sent_bin[(right - 1) % SEARCH_SENT_BINS] - ca->search.sent_bin[left % SEARCH_SENT_BINS];
	
	if (left == 0) /* If we are interpolating using the very first bin, the "previous" bin value is 0. */
		sent += (ca->search.sent_bin[left % SEARCH_SENT_BINS]) * fraction / 100;
	else
		sent += (ca->search.sent_bin[left % SEARCH_SENT_BINS] - ca->search.sent_bin[(left - 1) % SEARCH_SENT_BINS]) * fraction / 100;

	sent += (ca->search.sent_bin[right % SEARCH_SENT_BINS] - ca->search.sent_bin[(right - 1) % SEARCH_SENT_BINS]) * (100 - fraction) / 100;

	return sent;
}

/*
 * SEARCH: Compute target congestion window for CWND rollback.
 *
 * Estimates the target CWND based on delivered bytes from a past RTT window
 * (BDP approximation) and prepares SEARCH to drain excess in-flight data.
 */
static void search_compute_target_cwnd(struct sock *sk, u32 now_us, u32 rtt_us)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);
	s32 cong_idx = 0;
	u32 initial_rtt = 0;
	u64 overshoot_bytes = 0;
	u64 overshoot_bytes_rescaled = 0;
	u32 overshoot_cwnd = 0;
	u32 rtt_bins = 0;
	

 	initial_rtt = ca->search.bin_duration_us * SEARCH_BINS * 10 / search_window_duration_factor;

 	/* Number of bins spanning ~1 RTT */
 	rtt_bins = (initial_rtt + ca->search.bin_duration_us - 1) / ca->search.bin_duration_us; /* ceil*/

 	cong_idx = ca->search.curr_idx - rtt_bins;

 	if (ca->search.curr_idx - cong_idx <= SEARCH_ACKED_BINS - 1){

	 	/* Calculate the overshoot based on the delivered bytes between cong_idx and the current index */
	 	overshoot_bytes = search_compute_delivered_window(sk, cong_idx, ca->search.curr_idx);

	 	overshoot_bytes_rescaled = overshoot_bytes << ca->search.scale_factor;

	 	overshoot_cwnd = overshoot_bytes_rescaled  / tp->mss_cache;

	 	ca->search.search_targeted_cwnd = max(overshoot_cwnd, (u32)TCP_INIT_CWND);
	}
}

/* Function to log ACK analysis information */
static void search_log_ack_info(struct sock *sk, u32 rtt_us, u8 slow_start_status)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);

	u32 now_us = bictcp_clock_us(sk);

	// if (ntohs(inet_sk(sk)->inet_dport) != debug_port && ntohs(inet_sk(sk)->inet_sport) != debug_port)
	// 	return;

	search_print_header(ca);
	printk(KERN_CONT "ACK_FUNC_INFO: [now %u] [total_byte_acked %llu] [rtt_us %u] [num_lost %u] [total_retrans %u] [cwnd_pkt %u] [ssthresh %u] [mss %u] [ss_status %u] [delivered_rate %u] [rate_interval_us %u] [sk_pacing_rate %lu] [snd_nxt %u] [snd_una %u] [app_limited %u] [rate_app_limited %u] [tp_delivered %u] [total_bytes_sent %llu] \n", 
		now_us, tp->bytes_acked, rtt_us, tp->lost_out, tp->total_retrans, tcp_snd_cwnd(tp), tp->snd_ssthresh, tp->mss_cache, slow_start_status, tp->rate_delivered, tp->rate_interval_us, sk->sk_pacing_rate, tp->snd_nxt, tp->snd_una, tp->app_limited, tp->rate_app_limited, tp->delivered, tp->bytes_sent);
}

/* Function to log SEARCH analysis information */
static void search_log_info(struct sock *sk, u32 rtt_us, u64 curr_delv_bytes, u64 prev_sent_bytes, s32 norm_diff)
{
	struct bictcp *ca = inet_csk_ca(sk);
	
	u32 now_us = bictcp_clock_us(sk);

	// if (ntohs(inet_sk(sk)->inet_dport) != debug_port && ntohs(inet_sk(sk)->inet_sport) != debug_port)
	// 	return;

	search_print_header(ca);
	printk(KERN_CONT "SEARCH[%u]_INFO: [now %u] [bin_duration %u] [rtt_us %u] [curr_delv %llu] [prev_sent %llu] [norm_100 %d] [scale_factor %u] [curr_idx %u]\n", 
	  SEARCH_VERSION, now_us, ca->search.bin_duration_us, rtt_us, curr_delv_bytes, prev_sent_bytes, norm_diff, ca->search.scale_factor, ca->search.curr_idx);
}

/**
 * search_update - Update SEARCH bins and evaluate slow start exit conditions
 *
 * This function handles the periodic updates of the SEARCH algorithm's bins,
 * which track delivered bytes over defined time intervals. It ensures that
 * bins are updated when their boundaries are reached and computes delivered
 * bytes for the current and previous windows. If the delivered bytes in the
 * current window are significantly less than in the previous window, it exits
 * the slow start phase based on the normalized difference in delivered bytes.
 *
 * Key steps:
 * - Initialize bins if they are not yet set.
 * - Update bins when the current time exceeds the bin boundary.
 * - Check if there are enough bins to calculate previous window delivered bytes.
 * - Compute delivered bytes for current and previous windows, considering
 *   fractional adjustments for RTT overlaps.
 * - Evaluate exit condition using normalized difference in delivered bytes
 *   and invoke slow start exit if the condition is met.
 */
static void search_update(struct sock *sk, u32 rtt_us)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);

	s32 prev_idx = 0;
	u64 curr_delv_bytes = 0; 
	u64 prev_sent_bytes = 0;
	s32 norm_diff = 0;
	u32 now_us = bictcp_clock_us(sk);
	u32 fraction = 0;
	u32 segs_acked = 0;
	u32 snd_cnt = 0;
	u32 new_cwnd = 0;


	/* If SEARCH is not in Drain phase*/
	if (ca->search.search_cwnd_reduction_to_target == 0) {

		/* by receiving the first ack packet, initialize bin duration and bin end time */
		if (ca->search.curr_idx < 0) {  //new_change: app_limited
			search_init_bins(sk, now_us, rtt_us);
			// logging information about SEARCH analysis
			search_log_info(sk, rtt_us, curr_delv_bytes, prev_sent_bytes, norm_diff);
			return;
		}

		if (now_us < ca->search.bin_end_us)
			return;

		/* reach or pass the bin boundary, update bins */
		search_update_bins(sk, now_us, rtt_us);

		/* check if there is enough bins after shift for computing previous window */
		prev_idx = ca->search.curr_idx - (rtt_us / ca->search.bin_duration_us);

		if (prev_idx >= SEARCH_BINS && (ca->search.curr_idx - prev_idx) < SEARCH_EXTRA_SENT_BINS - 1) {

			/* Calculate delivered bytes for the current and previous windows */
			curr_delv_bytes = search_compute_delivered_window(sk,
									  ca->search.curr_idx - SEARCH_BINS,
									  ca->search.curr_idx);

			fraction = ((rtt_us % ca->search.bin_duration_us) * 100 / ca->search.bin_duration_us);

			prev_sent_bytes = search_compute_sent_window(sk,
									  prev_idx - SEARCH_BINS,
									  prev_idx,
									  fraction);

			if (prev_sent_bytes > 0) {
				norm_diff = (prev_sent_bytes - curr_delv_bytes) * 100 / prev_sent_bytes;

				/* check for exit condition */
				if ((2 * prev_sent_bytes) >= curr_delv_bytes && norm_diff >= search_thresh){

					/* Compute target cwnd but do NOT apply it yet */
					search_compute_target_cwnd(sk, now_us, rtt_us);
					/* Enable SEARCH Drain phase*/
					ca->search.search_cwnd_reduction_to_target = 1;
					ca->search.prior_delivered = tp->delivered;
				}
			}
		}
	}

	/* SEARCH drain phase */
	else {
		segs_acked = tp->delivered - ca->search.prior_delivered;
		ca->search.prior_delivered = tp->delivered;

		ca->search.search_drain_ackedseg += segs_acked;

		/* 
		 * Pace CWND increase during SEARCH drain.
		 * CWND grows only after a threshold number of ACKed segments
		 * to ensure controlled draining toward the target CWND.
		 */
		if (ca->search.search_drain_ackedseg >= SEARCH_DRAIN_ACKEDSEG_THRESH) {
		    snd_cnt = ca->search.search_drain_ackedseg / SEARCH_DRAIN_ACKEDSEG_THRESH;
		    ca->search.search_drain_ackedseg %= SEARCH_DRAIN_ACKEDSEG_THRESH;
		}

		new_cwnd = max(tcp_packets_in_flight(tp) + snd_cnt, ca->search.search_targeted_cwnd);

		tcp_snd_cwnd_set(tp, new_cwnd);   // like tcp_cwnd_reduction() in tcp_input.c

		/* Drain completed: lock CWND and exit slow start */
		if (tcp_snd_cwnd(tp) == ca->search.search_targeted_cwnd){
			tp->snd_ssthresh = tcp_snd_cwnd(tp);
			bictcp_search_reset(sk, RESET_BIN_DURATION_TRUE);
		}

	}

	// logging information about SEARCH analysis
	search_log_info(sk, rtt_us, curr_delv_bytes, prev_sent_bytes, norm_diff);
}

//////////////////////////////////////////////////////////////
__bpf_kfunc static void cubictcp_acked(struct sock *sk, const struct ack_sample *sample)
{
	const struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);
	u32 delay;

	/* Some calls are for duplicates without timetamps */
	if (sample->rtt_us < 0)
		return;

	/* Discard delay samples right after fast recovery */
	if (ca->epoch_start && (s32)(tcp_jiffies32 - ca->epoch_start) < HZ)
		return;

	delay = sample->rtt_us;
	if (delay == 0)
		delay = 1;

	/* first time call or link delay decreases */
	if (ca->delay_min == 0 || ca->delay_min > delay)
		ca->delay_min = delay;

	//////////////////////// SEARCH ////////////////////////
	if (tcp_in_slow_start(tp)) {
		if (slow_start_mode == SS_SEARCH) {
			/* implement search algorithm */
			search_update(sk, delay);
		} else if (slow_start_mode == SS_HYSTART && !ca->hystart.found)
			hystart_update(sk, delay);
	}

	/////////////////logging-ACK information/////////////////
	if (tcp_in_slow_start(tp))
		search_log_ack_info(sk, delay, 1);
	else 
		search_log_ack_info(sk, delay, 2);
}

static struct tcp_congestion_ops cubictcp __read_mostly = {
	.init		= cubictcp_init,
	.ssthresh	= cubictcp_recalc_ssthresh,
	.cong_avoid	= cubictcp_cong_avoid,
	.set_state	= cubictcp_state,
	.undo_cwnd	= tcp_reno_undo_cwnd,
	.cwnd_event	= cubictcp_cwnd_event,
	.pkts_acked     = cubictcp_acked,
	.owner		= THIS_MODULE,
	.name		= "cubic_search",
};

BTF_KFUNCS_START(tcp_cubic_check_kfunc_ids)
BTF_ID_FLAGS(func, cubictcp_init)
BTF_ID_FLAGS(func, cubictcp_recalc_ssthresh)
BTF_ID_FLAGS(func, cubictcp_cong_avoid)
BTF_ID_FLAGS(func, cubictcp_state)
BTF_ID_FLAGS(func, cubictcp_cwnd_event)
BTF_ID_FLAGS(func, cubictcp_acked)
BTF_KFUNCS_END(tcp_cubic_check_kfunc_ids)

static const struct btf_kfunc_id_set tcp_cubic_kfunc_set = {
	.owner = THIS_MODULE,
	.set   = &tcp_cubic_check_kfunc_ids,
};

static int __init cubictcp_register(void)
{
	int ret;

	BUILD_BUG_ON(sizeof(struct bictcp) > ICSK_CA_PRIV_SIZE);

	/* Precompute a bunch of the scaling factors that are used per-packet
	 * based on SRTT of 100ms
	 */

	beta_scale = 8*(BICTCP_BETA_SCALE+beta) / 3
		/ (BICTCP_BETA_SCALE - beta);

	cube_rtt_scale = (bic_scale * 10);	/* 1024*c/rtt */

	/* calculate the "K" for (wmax-cwnd) = c/rtt * K^3
	 *  so K = cubic_root( (wmax-cwnd)*rtt/c )
	 * the unit of K is bictcp_HZ=2^10, not HZ
	 *
	 *  c = bic_scale >> 10
	 *  rtt = 100ms
	 *
	 * the following code has been designed and tested for
	 * cwnd < 1 million packets
	 * RTT < 100 seconds
	 * HZ < 1,000,00  (corresponding to 10 nano-second)
	 */

	/* 1/c * 2^2*bictcp_HZ * srtt */
	cube_factor = 1ull << (10+3*BICTCP_HZ); /* 2^40 */

	/* divide by bic_scale and by constant Srtt (100ms) */
	do_div(cube_factor, bic_scale * 10);

	ret = register_btf_kfunc_id_set(BPF_PROG_TYPE_STRUCT_OPS, &tcp_cubic_kfunc_set);
	if (ret < 0)
		return ret;
	return tcp_register_congestion_control(&cubictcp);
}

static void __exit cubictcp_unregister(void)
{
	tcp_unregister_congestion_control(&cubictcp);
}

module_init(cubictcp_register);
module_exit(cubictcp_unregister);

MODULE_AUTHOR("Sangtae Ha, Stephen Hemminger");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CUBIC TCP");
MODULE_VERSION("2.3");


// use sent_bytes
// App_limited is removed
/* use delivered (in packet) for cwnd set in drain (delivered consider 
	retransmition). ( I can also instead use tp->bytes_acked / mss) */