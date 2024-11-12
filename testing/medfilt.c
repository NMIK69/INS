#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>


#define ARR_SIZE(arr) (sizeof(arr) / sizeof(*arr))

#define WIN_SIZE 5
#define WIN_MID (WIN_SIZE / 2)
#define NUM_DEV 10
#define MAGIC_VAL 1.4826f

#define SWAP(a, b, swp)\
	swp = a;\
	a = b;\
	b = swp;

#define SORT_THREE(arr, swp)\
	if (arr[0] > arr[1]) {\
		SWAP(arr[0],arr[1], swp)\
	}\
	if (arr[1] > arr[2]) {\
		SWAP(arr[1],arr[2], swp)\
	}\
	if (arr[0] > arr[1]) {\
		SWAP(arr[0],arr[1], swp)\
	}

struct record
{
	float ax, ay, az;
	float wx, wy, wz;
};


struct med_res
{
	int is_outlier;
	float median;
};


struct med_res median_filter(float a, float b, float c)
{
	float tmp, median, mad;
	static float arr[3];
	static float adev[3];

	arr[0] = a;
	arr[1] = b;
	arr[2] = c;

	SORT_THREE(arr, tmp);
	median = arr[1];

	adev[0] = fabs(a - median);	
	adev[1] = fabs(b - median);	
	adev[2] = fabs(c - median);	

	SORT_THREE(adev, tmp);
	mad = adev[1];

	//printf("a: %f, b: %f, c: %f\n", a, b, c);
	//
	//for(int i = 0; i < 3; i++) {
	//	printf("arr[%d]: %f\n", i, arr[i]);
	//}

	//for(int i = 0; i < 3; i++) {
	//	printf("adev[%d]: %f\n", i, adev[i]);
	//}

	struct med_res res = {.is_outlier = 0, .median = 0};

	float diff = fabs(b - median);
	float cap = (NUM_DEV * mad * MAGIC_VAL);
	//printf("diff: %f\n", diff);
	//printf("cap: %f\n", cap);

	assert(cap >= 0.0f);

	if(diff > cap) {
	//if(diff > cap) {
		res.is_outlier = 1;
		res.median = median;
		//return median;
	}
	//else if(fabs(b) > 0.1) {
	//	printf("a: %f, b: %f, c: %f\n", a, b, c);
	//	
	//	for(int i = 0; i < 3; i++) {
	//		printf("arr[%d]: %f\n", i, arr[i]);
	//	}

	//	for(int i = 0; i < 3; i++) {
	//		printf("adev[%d]: %f\n", i, adev[i]);
	//	}
	//	printf("diff: %f\n", diff);
	//	printf("cap: %f\n", cap);
	//}
	return res;
	//return b;
}

static ssize_t get_nlines(const char *fname)
{
	char entry[1024];
	FILE *f = fopen(fname, "r");

	if(f == NULL)
		return -1;

	ssize_t count = 0;

	while(feof(f) == 0 && fgets(entry, ARR_SIZE(entry), f) != NULL) {
		count += 1;
	}

	fclose(f);
	return count;
}

struct record *read_file(const char *fname, size_t *len)
{
	char entry[1024];
	FILE *f = fopen(fname, "r");
	assert(f != NULL);

	ssize_t nlines = get_nlines(fname);
	assert(nlines != -1);

	struct record *rec = malloc(sizeof(*rec) * nlines);
	if(rec == NULL)
		return NULL;
	
	size_t i = 0;
	int ts;
	while(feof(f) == 0 && fgets(entry, ARR_SIZE(entry), f) != NULL) {
		
		int ret = sscanf(entry, "%d,%f,%f,%f,%f,%f,%f", 
				&ts,
				&rec[i].ax, &rec[i].ay, &rec[i].az,
				&rec[i].wx, &rec[i].wy, &rec[i].wz);
		assert(ret == 7);

		i += 1;
	}

	(*len) = i;
	fclose(f);
	return rec;
}

void write_file(const char *fname, struct record *rec, size_t len)
{
	FILE *f = fopen(fname, "w");
	assert(f != NULL);

	int ts = 0;
	for(size_t i = 0; i < len; i++) {
		int ret = fprintf(f, "%d,%f,%f,%f,%f,%f,%f\n", 
				ts,
				rec[i].ax, rec[i].ay, rec[i].az,
				rec[i].wx, rec[i].wy, rec[i].wz);

		assert(ferror(f) == 0 && ret > 0);
	}

	fclose(f);
}


int main(void)
{
	struct record *rec;
	size_t rec_len;
	int start, mid, end;
	size_t i;
	struct med_res res;

	rec = read_file("imu_test.txt", &rec_len);
	assert(rec != NULL);

	start = 0;
	mid = WIN_MID;
	end = mid + WIN_MID;

	i = 0;
	while((size_t)end < rec_len) {
		
		res = median_filter(rec[start].ax,
				    rec[mid].ax,
				    rec[end].ax);

		if(res.is_outlier == 1) {
			//printf("Hit at %zu. From %f to %f\n",
			//	i, rec[mid].ax, res.median);
			rec[mid].ax = res.median;
		}

		res = median_filter(rec[start].ay,
				    rec[mid].ay,
				    rec[end].ay);

		if(res.is_outlier == 1) {
			//printf("Hit at %zu. From %f to %f\n",
			//	i, rec[mid].ay, res.median);
			rec[mid].ay = res.median;
		}

		res = median_filter(rec[start].az,
				    rec[mid].az,
				    rec[end].az);

		if(res.is_outlier == 1) {
			//printf("Hit at %zu. From %f to %f\n",
			//	i, rec[mid].az, res.median);
			rec[mid].az = res.median;
		}

		start += 1;
		mid += 1;
		end += 1;
		i += 1;
	}
	
	write_file("filt.txt", rec, rec_len);

	return 0;
}
