#pragma once
class Servo {
    int pos=0;
public:
    void attach(int){}
    void write(int p){pos=p;}
    int read(){return pos;}
};
