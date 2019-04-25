#include <stdio.h>
#include <sys/timeb.h>

void mdelay(int msec)
{
	unsigned long start, end;
	struct timeb tp;

	ftime(&tp);
	start = tp.time * 1000 + tp.millitm;
	end = start + msec;
	while (start < end) {
		ftime(&tp);
		start = tp.time * 1000 + tp.millitm;
	}
}


