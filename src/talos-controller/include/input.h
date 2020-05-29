#include <pthread.h>
#include <time.h>

char buf = 10;
pthread_mutex_t pc_mutex;
pthread_cond_t pc_condp, pc_condc;

void *readInputThread(void * nul)
{
    while(true){
        char tmp = getchar();
        pthread_mutex_lock(&pc_mutex);
        while(buf != 10)
            pthread_cond_wait(&pc_condp, &pc_mutex);
        buf = tmp;
        pthread_cond_signal(&pc_condc);
        pthread_mutex_unlock(&pc_mutex);
    }
    pthread_exit(NULL);
}

void initInput(){
    pthread_t thread;
    pthread_attr_t attr;
    pthread_mutex_init(&pc_mutex, NULL);
    pthread_cond_init(&pc_condp, NULL);
    pthread_cond_init(&pc_condc, NULL);
    pthread_attr_init(&attr);
    pthread_create(&thread, &attr, readInputThread, NULL);
    pthread_detach(thread);
}

void destoryInput(){
    pthread_mutex_destroy(&pc_mutex);
    pthread_cond_destroy(&pc_condc);
    pthread_cond_destroy(&pc_condp);
}

char readInput(unsigned int seconds){  
    pthread_mutex_lock(&pc_mutex);
    if(buf == 10){
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += seconds;
        pthread_cond_timedwait(&pc_condc, &pc_mutex, &ts);
    }
    char tmp = buf;
    buf = 10;
    pthread_cond_signal(&pc_condp);
    pthread_mutex_unlock(&pc_mutex);
    return tmp;
}
