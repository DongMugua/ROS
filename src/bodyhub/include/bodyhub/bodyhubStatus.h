#ifndef BODYHUBSTATUS_H
#define BODYHUBSTATUS_H

namespace StatusEnum {

enum StatusStyle {
  isHeadQEmpty = 1,
  isMotoQEmpty

};
}

namespace StateEnum {

enum StateStyle {
  init = 20,
  preReady,
  ready,
  running,
  pause,
  stoping,
  error,
  directOperate,
  walking
};
}

#endif