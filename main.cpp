
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#include <cstdlib>
#include <array>
#include <iostream>
#include <string>
#include <vector>

constexpr int PITCH_JOINT_ID = 2;
constexpr int ROLL_JOINT_ID = 1;
constexpr int YAW_JOINT_ID = 0;


class Module : public yarp::os::RFModule {
private:
    yarp::dev::PolyDriver driver;
    yarp::dev::IEncoders* iEnc{ nullptr };
    yarp::dev::IPidControl* iPidCtrl{ nullptr };
    yarp::dev::IPositionControl* iPosCtrl{ nullptr };
    std::array<double, 3> refs;
    std::array<double, 3> errs;
    std::array<double, 3> currs;
    yarp::os::Port port_trajectory;
    yarp::os::Port port_ref;
    yarp::os::Port port_outs;
    yarp::os::Bottle bot_traj;
    yarp::os::Bottle bot_ref;
    yarp::os::Bottle bot_out;
    yarp::os::BufferedPort<yarp::os::Bottle> input;
    
   std::array<double, 3> pidKs;

    
    bool configure(yarp::os::ResourceFinder & rf) {        
        yarp::os::Property conf;
        conf.put("device", "remote_controlboard");
        conf.put("remote", rf.check("remote", yarp::os::Value("/nfa/wrist_mc")).asString());
        conf.put("local", "/logger");

        if (!driver.open(conf)) {
            yError() << "Failed to connect to" << conf.find("remote").asString();
            return false;
        }

        if (!(driver.view(iEnc))) {
            yError() << "Failed to open encoder interface";
            driver.close();
            return false;
        }

        if (!(driver.view(iPidCtrl))) {
            yError() << "Failed to open PID control interface";
            driver.close();
            return false;
        }

        if (!(driver.view(iPosCtrl))) {
            yError() << "Failed to open position control interface";
            driver.close();
            return false;
        }
        
        if(!port_outs.open("/CbWrist/pidout:o") ||
            !port_ref.open("/CbWrist/refs:o") ||
            !port_trajectory.open("/CbWrist/traj:o")) {
            yError() << "Could not open one or more ports!";
            return false;            
        }
        
        input.open("/CbWrist/pidK:i");
        
        return true;
    }
    
    bool updateModule() {
        iPidCtrl->getPidReferences(yarp::dev::VOCAB_PIDTYPE_POSITION, refs.data());
        iPidCtrl->getPidErrors(yarp::dev::VOCAB_PIDTYPE_POSITION, errs.data());
        iPidCtrl->getPidOutputs(yarp::dev::VOCAB_PIDTYPE_POSITION, currs.data());
        
        for(uint8_t i = 0; i < 3; ++i) {
            bot_out.addFloat64(currs[i]);
            bot_ref.addFloat64(refs[i]);
            bot_traj.addFloat64(refs[i] - errs[i]);
        }
        
//        yInfo() << "ref" << refs[0] <<refs[1] <<refs[2]; 
//        yInfo() << "cur" << currs[0] <<currs[1] <<currs[2]; 
//        yInfo() << "err" << errs[0] <<errs[1] <<errs[2]; 

        if(!port_outs.write(bot_out))
            yError() << "Could not write to port" << port_outs.getName();
        if(!port_trajectory.write(bot_traj)) 
            yError() << "Could not write to port" << port_trajectory.getName();
        if(!port_ref.write(bot_ref)) 
            yError() << "Could not write to port" << port_ref.getName();
        
        bot_out.clear();
        bot_traj.clear();
        bot_ref.clear();
        
        
        //yInfo() << "before read";
        yarp::os::Bottle *reader = input.read(false);
        //yInfo() << "before read 2";

        if(reader) {                
            for(uint8_t i = 0; i < 3; ++i) { 
                pidKs[i] = reader->get(i).asFloat64();
            }
            
            yInfo() << "Received pid gains: (Kp Kd Ki) (" << pidKs[0] 
                    << pidKs[1] << pidKs[2] << ")";
            

            std::array<yarp::dev::Pid, 3> p;
            iPidCtrl->getPids(yarp::dev::VOCAB_PIDTYPE_POSITION, p.data());
            
            for(uint8_t i = 0; i < 3; ++i) {
                p[i].setKp(pidKs[0]);
                p[i].setKd(pidKs[1]);
                p[i].setKi(pidKs[2]);
            }
            
            iPidCtrl->setPids(yarp::dev::VOCAB_PIDTYPE_POSITION, p.data());
            
            yInfo() << "Set pid params";
        }
        //yInfo() << "after";
        
        return true;
    }
    
    double getPeriod() {
        return 0.1;
    };
    
    bool close() {
        driver.close();
//         yInfo() << "Closing connections";
//         
//         yarp::os::Network::disconnect(port_outs.getName(), "/scope/pidout:i");
//         yarp::os::Network::disconnect(port_ref.getName(), "/scope/refs:i");
//         yarp::os::Network::disconnect(port_trajectory.getName(), "/scope/traj:i");
//                 
        return true;
    }
    
public:
    Module(){}
    ~Module(){}
};

int main(int argc, char **argv) {

    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "Unable to find YARP server!";
        return EXIT_FAILURE;
    }
    
    Module m;
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    return m.runModule(rf);
}
