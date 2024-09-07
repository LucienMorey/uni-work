#include "databuffer.h"

DataBuffer::DataBuffer(){
    values.clear();
}

bool DataBuffer::removeValues(double min, double max){
    bool removed=false;
    buffer_mutex_.lock();

    auto it = values.begin();

    while ( it != values.end()) {
        if (*it < min || *it > max) {
            values.erase(it);
            removed=true;
        } else {
            it++;
        }
    }
    buffer_mutex_.unlock();
    return removed;
}

void DataBuffer::addValue(double value){
  buffer_mutex_.lock();
  values.push_back(value);
  buffer_mutex_.unlock();
}

unsigned int DataBuffer::trimLength(unsigned int value){

  buffer_mutex_.lock();

  while (values.size()>value) {
      values.erase(values.begin());
  }
  buffer_mutex_.unlock();
}
