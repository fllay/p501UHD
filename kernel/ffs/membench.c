// Read bandwidth of a CMA buffer through /dev/mem: uncached (O_SYNC) vs cached + DC CIVAC per cache line.
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static void civac(void *p, size_t n) { for (uintptr_t a = (uintptr_t)p & ~63UL; a < (uintptr_t)p + n; a += 64) asm volatile("dc civac, %0" :: "r"(a) : "memory"); asm volatile("dsb sy" ::: "memory"); }
int main(int argc, char **argv)
{
	uint64_t phys = strtoull(argv[1], 0, 0); size_t size = strtoull(argv[2], 0, 0); int cached = atoi(argv[3]);
	int fd = open("/dev/mem", O_RDWR | (cached ? 0 : O_SYNC)); if (fd < 0) { perror("open"); return 1; }
	void *m = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys); if (m == MAP_FAILED) { perror("mmap"); return 1; }
	char *dst = aligned_alloc(4096, size); memset(dst, 1, size);
	size_t chunk = 1 << 20; double t0 = now(); size_t tot = 0;
	for (int rep = 0; rep < 8; rep++) for (size_t o = 0; o + chunk <= size; o += chunk) {
		if (cached) civac((char *)m + o, chunk);
		memcpy(dst + o, (char *)m + o, chunk); tot += chunk; }
	double dt = now() - t0;
	printf("%s: %.0f MB/s (%.1f MB in %.2fs)\n", cached ? "cached+civac" : "uncached O_SYNC", tot / dt / 1e6, tot / 1e6, dt);
	return 0;
}
