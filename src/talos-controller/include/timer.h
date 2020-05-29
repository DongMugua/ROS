#include <time.h>
#include <unistd.h>

#ifndef SLEEPFUNC_H
#define SLEEPFUNC_H
#define sleep(x) nSleep(x)
#define usleep(x) u_nSleep(x)
inline int __nsleep(const struct timespec *req, struct timespec *rem)
{
    struct timespec temp_rem;
    if(nanosleep(req,rem)==-1)
        __nsleep(rem,&temp_rem);
    else
        return 1;
}

inline unsigned int nSleep(unsigned int seconds)
{
    struct timespec req = {0}, rem = {0};
    time_t sec = seconds;
    req.tv_sec = sec;
    req.tv_nsec = 0;
    __nsleep(&req, &rem);
    return 0;
}

inline int u_nSleep(useconds_t uSeconds)
{
    struct timespec req = {0}, rem = {0};
    time_t sec = (int)(uSeconds/1000000);
    uSeconds = uSeconds-(sec*1000000);
    req.tv_sec = sec;
    req.tv_nsec = uSeconds*1000L;
    __nsleep(&req, &rem);
    return 0;
}
#endif


