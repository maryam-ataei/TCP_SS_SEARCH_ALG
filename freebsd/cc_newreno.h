/*-
 * Copyright (c) 2017 Tom Jones <tj@enoti.me>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

 #ifndef _CC_NEWRENO_H
 #define _CC_NEWRENO_H
 
 /* SEARCH_begin */
 #define CCALGONAME_NEWRENO "newreno_search"
 #define SEARCH_WINDOW_SIZE_FACTOR 35
 #define SEARCH_BINS 10
 #define SEARCH_EXTRA_BINS 15
 #define SEARCH_TOTAL_BINS (SEARCH_BINS + SEARCH_EXTRA_BINS)
 #define SEARCH_THRESH 35
 #define MAX_US_INT 0xffff
 #define SEARCH_ALPHA MAX_US_INT // If missed bins > SEARCH_ALPHA bins, SEARCH reset (This is in progress...)
 #define V_CWND_ROLLBACK 0

 typedef uint32_t search_bin_t;
 
 enum reset_bin_duration { 
	 RESET_BIN_DURATION_TRUE,    // Reset bin duration   
	 RESET_BIN_DURATION_FALSE    // Do not reset bin duration
 };
 /* SEARCH_end */
 
 
 struct newreno {
	 uint32_t beta;
	 uint32_t beta_ecn;
	 uint32_t newreno_flags;
	 uint32_t css_baseline_minrtt;
	 uint32_t css_current_round_minrtt;
	 uint32_t css_lastround_minrtt;
	 uint32_t css_rttsample_count;
	 uint32_t css_entered_at_round;
	 uint32_t css_current_round;
	 uint32_t css_fas_at_css_entry;
	 uint32_t css_lowrtt_fas;
	 uint32_t css_last_fas;
 
	 /* SEARCH_begin */
	 uint32_t search_bin_duration_us;                /* duration of each bin in microsecond */
	 int32_t  search_curr_idx;                       /* total number of bins */
	 uint64_t search_bin_end_us;                     /* end time of the latest bin in microseconds */
	 search_bin_t search_bin[SEARCH_TOTAL_BINS];	 /* array to keep bytes for bins */
	 uint8_t search_scale_factor;		         /* scale factor to fit the value with bin size */
	 uint32_t search_bytes_this_bin;		 /* bytes_acked during this bin */
 	 uint32_t search_reset_flag;
 };
 
 #define SEARCH_BIN(ccv, index) ((struct newreno*)(ccv)->cc_data)->search_bin[(index) % SEARCH_TOTAL_BINS]
 /* SEARCH_end */

 struct cc_newreno_opts {
	 int		name;
	 uint32_t	val;
 };
 
 #define CC_NEWRENO_BETA			1	/* Beta for normal DUP-ACK/Sack recovery */
 #define CC_NEWRENO_BETA_ECN		2	/* ECN Beta for Abe */
 
 /* Flags values */
 #define CC_NEWRENO_HYSTART_ENABLED	0x0002	/* We can do hystart, a loss removes this flag */
 #define CC_NEWRENO_HYSTART_IN_CSS	0x0004	/* If we enter hystart CSS this flag is set */
 #define CC_NEWRENO_BETA_ECN_ENABLED	0x0020
 #endif /* _CC_NEWRENO_H */
