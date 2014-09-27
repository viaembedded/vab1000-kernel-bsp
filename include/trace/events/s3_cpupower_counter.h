/*
 * include/trace/events/s3_cpupower_counter.h
 *
 *
 * Copyright (c) 2010-2013, S3 Graphics IncCorporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM s3trace

//#if !defined(_TRACE_S3_H) || defined(TRACE_HEADER_MULTI_READ)
//#define _TRACE_S3_H

#ifndef _TRACE_S3_H
#define _TRACE_S3_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>

char  trace_counter_name[20];
s64   trace_counter;

DEFINE_TRACE(s3_counter_output);

DECLARE_EVENT_CLASS(s3,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(__field(const char *, name)),
	TP_fast_assign(__entry->name = name;),
	TP_printk("name=%s", __entry->name)
);


TRACE_EVENT(s3_counter_output,
	TP_PROTO(const char *name, s64 count, int cpu),

	TP_ARGS(name, count, cpu),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(s64, count)
		__field(int, cpu)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->count = count;
		__entry->cpu = cpu;
	),

	TP_printk("name=%s, count=%lu, cpu=%d", __entry->name, __entry->count, __entry->cpu)
);


#endif /*  _TRACE_S3_H */
/* This part must be outside protection */
//#include <trace/define_trace.h>
