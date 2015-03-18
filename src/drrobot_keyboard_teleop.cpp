/*!
 * drrobot_keyboard_teleop.cpp
 * Copyright (c) 2011, Dr Robot Inc
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!

@mainpage
  drrobot_keyboard_teleop for demonstration and testing published geometry_msgs/Twist message to drrobot_player.
  It will use 4 keys to control robot move around
  a/A -- 0.5/1 full speed turn to left
  w/W -- 0.5/1 full speed forward
  d/D -- 0.5/1 full speed turn to right
  s/S -- 0.5/1 full speed backward
  if no key pressed, it will stop robot
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
          then run drrobot_player first.
@verbatim
$ drrobot_keyboard_teleop
@endverbatim

<hr>
@section topic ROS topics

Publishes to (name / type):
-@b drrobot_cmd_vel: will publish drrobot_cmd_vel Message to drrobot_player. For robot from Dr Robot Inc, we only need provide linear.x
    as going forward/backward speed, and angular.z as turning speed. drrobot_player will transform these command value to encoder control
    command value and send them to motion control system on the robot
<hr>
*/
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>


#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_W_CAP 0x57
#define KEYCODE_A_CAP 0x41
#define KEYCODE_S_CAP 0x53
#define KEYCODE_D_CAP 0x44

#define KEYCODE_I 0x69
#define KEYCODE_K 0x6b
#define KEYCODE_O 0x6f
#define KEYCODE_L 0x6c

#define KEYCODE_I_CAP 0x49
#define KEYCODE_K_CAP 0x4b
#define KEYCODE_O_CAP 0x4f
#define KEYCODE_L_CAP 0x4c

#define KEYCODE_Q_CAP 0x51
#define KEYCODE_P_CAP 0x50
#define KEYCODE_Q 0x71
#define KEYCODE_P 0x70


class DrRobotKeyboardTeleopNode
{
    private:
        geometry_msgs::Twist cmdvel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        DrRobotKeyboardTeleopNode()
        {
            pub_ = n_.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 1);
            ros::NodeHandle n_private("~");
        }

        ~DrRobotKeyboardTeleopNode() { }
        void keyboardLoop();

        void stopRobot()
        {
	    //for wheel control
            cmdvel_.linear.x = 0.0;
            cmdvel_.angular.z = 0.0;
	    // for arm control
	    cmdvel_.linear.y = 0.0;
	    cmdvel_.linear.z  =0.0;
            pub_.publish(cmdvel_);
        }
};

DrRobotKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"drrobot_teleope_keyboard", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    DrRobotKeyboardTeleopNode tbk;

    boost::thread t = boost::thread(boost::bind(&DrRobotKeyboardTeleopNode::keyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void DrRobotKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double maxVel = 1.0;
    double maxTurn = 1.0;
    bool dirty = false;
    double armCmd1 = 0;
    double armCmd2 = 0;


    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Use I/K keys to control the arm1");
    puts("Use O/L keys to control the arm2");
    puts("Press Shift to move faster");

    puts("Press Q/q to stop robot");
    puts("Press P/p to stop arm");
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                //stopRobot();
                dirty = false;
            }

            continue;
        }

        switch(c)
        {
            case KEYCODE_I:
		armCmd1 = 0.5;
		armCmd2 = 0;
		maxVel = 0;
                maxTurn = 0;
		dirty = true;
		break;
	    case KEYCODE_I_CAP:
		armCmd1 = 1.0;
		armCmd2 = 0;
		maxVel = 0;
                maxTurn = 0;
		dirty = true;
		break;
	    case KEYCODE_K_CAP:
		armCmd1 = -1.0;
		armCmd2 = 0;
		maxVel = 0;
                maxTurn = 0;
		dirty = true;
		break;
	    case KEYCODE_K:
		armCmd1 = -0.5;
		armCmd2 = 0;
		maxVel = 0;
                maxTurn = 0;
		dirty = true;
		break;

            case KEYCODE_O:
		armCmd2 = 0.5;
		armCmd1 = 0;
		maxVel = 0;
                maxTurn = 0;
		dirty = true;
		break;
	    case KEYCODE_O_CAP:
		armCmd2 = 1.0;
		armCmd1 = 0;
		maxVel = 0;
                maxTurn = 0;
		dirty = true;
		break;
	    case KEYCODE_L_CAP:
		armCmd2 = -1.0;
		armCmd1 = 0;
		maxVel = 0;
                maxTurn = 0;
		dirty = true;
		break;
	    case KEYCODE_L:
		armCmd2 = -0.5;
		armCmd1 = 0;
		maxVel = 0;
                maxTurn = 0;
		dirty = true;
		break;

            case KEYCODE_W:
		armCmd2 = 0;
		armCmd1 = 0;
                maxVel = 0.5;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_S:
		armCmd2 = 0;
		armCmd1 = 0;
                maxVel = -0.5;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_A:
		armCmd2 = 0;
		armCmd1 = 0;
                maxVel = 0;
                maxTurn = 0.5;
                dirty = true;
                break;
            case KEYCODE_D:
		armCmd2 = 0;
		armCmd1 = 0;
                maxVel = 0;
                maxTurn = -0.5;
                dirty = true;
                break;
            case KEYCODE_W_CAP:
		armCmd2 = 0;
		armCmd1 = 0;
                maxVel = 1.0;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_S_CAP:
		armCmd2 = 0;
		armCmd1 = 0;
                maxVel = -1.0;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_A_CAP:
		armCmd2 = 0;
		armCmd1 = 0;
                maxVel = 0;
                maxTurn = 1.0;
                dirty = true;
                break;
            case KEYCODE_D_CAP:
		armCmd2 = 0;
		armCmd1 = 0;
                maxVel = 0;
                maxTurn = -1.0;
                dirty = true;
                break;
	    case KEYCODE_Q_CAP:
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
	    case KEYCODE_Q:
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
	    case KEYCODE_P_CAP:
		armCmd2 = 0;
		armCmd1 = 0;
                dirty = true;
                break;
	    case KEYCODE_P:
		armCmd2 = 0;
		armCmd1 = 0;
                dirty = true;
                break;


            default:
                maxVel = 0;
                maxTurn = 0;
		armCmd1 = 0;
		armCmd2 = 0;
                dirty = false;
        }

        cmdvel_.linear.x = maxVel;
        cmdvel_.angular.z = maxTurn;
	cmdvel_.linear.y = armCmd1;
	cmdvel_.linear.z = armCmd2;
        ROS_INFO("Send control command [ %f, %f]", cmdvel_.linear.x, cmdvel_.angular.z);
	ROS_INFO("Send arm control command [ %f, %f]", cmdvel_.linear.y, cmdvel_.linear.z);
        pub_.publish(cmdvel_);
	
    }
}

