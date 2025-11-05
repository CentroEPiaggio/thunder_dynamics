#ifndef THUNDERCONTROL_H
#define THUNDERCONTROL_H

// namespace thunder_ns{

template<class Robot> class thunder_control{
    private:
        Robot* robot;

    public:
        thunder_control(Robot* robot);
        // ~thunder_control();

        int init();
        int start();
        int update();
};
	
// }


#endif