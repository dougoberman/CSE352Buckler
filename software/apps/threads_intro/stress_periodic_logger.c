
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

#define LOG_INTERVAL_MS 1000
#define RUNTIME_SECONDS 100
#define DELAY_THRESH 5

void sleep_ms(int ms) {
    struct timespec ts = {ms / 1000, (ms % 1000) * 1000000};
    nanosleep(&ts, NULL);
}

void* logger_thread(void* arg) {
    
    /* Implement a thread function that runs every LOG_INTERVAL_MS
    Print a message if it misses an interval by DELAY_THRESH */
    return NULL;
}

void* stress_thread(void* arg) {
    
    /* Another thread function that exacerbates the timing of the logger thread */

    return NULL;
}


int main() {

	/* necessary commands for thread creation */
    return 0;
}
