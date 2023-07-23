/*
 *	wiiuse
 *
 *	Written By:
 *		Michael Laforest	< para >
 *		Email: < thepara (--AT--) g m a i l [--DOT--] com >
 *
 *	Copyright 2006-2007
 *
 *	This file is part of wiiuse.
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *	$Header$
 *
 */


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

#include <stdio.h> /* for printf */

#include "keymap.h"
#include "wiiuse.h" /* for wiimote_t, classic_ctrl_t, etc */

#ifndef WIIUSE_WIN32
#include <unistd.h> /* for usleep */
#endif

#define MAX_WIIMOTES 4
sensor_msgs::msg::Joy joy_msg;
using namespace std::chrono_literals;

/**
 *	@brief Callback that handles an event.
 *
 *	@param wm		Pointer to a wiimote_t structure.
 *
 *	This function is called automatically by the wiiuse library when an
 *	event occurs on the specified wiimote.
 */
void handle_event(struct wiimote_t* wm) {
    wiiuse_motion_sensing(wm, 1);
    printf("\n\n--- EVENT [id %i] ---\n", wm->unid);

    /* if a button is pressed, report it */
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_A)) {
        printf("A pressed\n");
        joy_msg.buttons[WIIMOTE_A] = 1;
    }
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_B)) {
        printf("B pressed\n");
        joy_msg.buttons[WIIMOTE_B] = 1;
    }
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_UP)) {
        printf("UP pressed\n");
        joy_msg.axes[WIIMOTE_DPAD_Y] = 1;
    }
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_DOWN)) {
        printf("DOWN pressed\n");
        joy_msg.axes[WIIMOTE_DPAD_Y] = -1;
    }
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_LEFT)) {
        printf("LEFT pressed\n");
        joy_msg.axes[WIIMOTE_DPAD_X] = 1;
    }
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_RIGHT)) {
        printf("RIGHT pressed\n");
        joy_msg.axes[WIIMOTE_DPAD_X] = -1;
    }
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_MINUS)) {
        printf("MINUS pressed\n");
        joy_msg.buttons[WIIMOTE_MINUS] = 1;
    }
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_PLUS)) {
        printf("PLUS pressed\n");
        joy_msg.buttons[WIIMOTE_PLUS] = 1;
    }
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_ONE)) {
        printf("ONE pressed\n");
        joy_msg.buttons[WIIMOTE_1] = 1;
    }
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_TWO)) {
        printf("TWO pressed\n");
        joy_msg.buttons[WIIMOTE_2] = 1;
    }
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_HOME)) {
        printf("HOME pressed\n");
        joy_msg.buttons[WIIMOTE_HOME] = 1;
    }

    if (WIIUSE_USING_ACC(wm)) {
        printf("wiimote roll  = %f [%f]\n", wm->orient.roll, wm->orient.a_roll);
        printf("wiimote pitch = %f [%f]\n", wm->orient.pitch, wm->orient.a_pitch);
        printf("wiimote yaw   = %f\n", wm->orient.yaw);
    }

    if (wm->exp.type == EXP_NUNCHUK || wm->exp.type == EXP_MOTION_PLUS_NUNCHUK) {
        /* nunchuk */
        struct nunchuk_t* nc = (nunchuk_t*)&wm->exp.nunchuk;

        if (IS_PRESSED(nc, NUNCHUK_BUTTON_C)) {
            printf("Nunchuk: C pressed\n");
            joy_msg.buttons[NUNCHUCK_C] = 1;
        }
        if (IS_PRESSED(nc, NUNCHUK_BUTTON_Z)) {
            printf("Nunchuk: Z pressed\n");
            joy_msg.buttons[NUNCHUCK_Z] = 1;
        }

        printf("nunchuk roll  = %f\n", nc->orient.roll);
        printf("nunchuk pitch = %f\n", nc->orient.pitch);
        printf("nunchuk yaw   = %f\n", nc->orient.yaw);

        printf("nunchuk joystick angle:     %f\n", nc->js.ang);
        printf("nunchuk joystick magnitude: %f\n", nc->js.mag);

        printf("nunchuk joystick vals:      %f, %f\n", nc->js.x, nc->js.y);
        joy_msg.axes[NUNCHUCK_STICK_X] = -(nc->js.x);
        joy_msg.axes[NUNCHUCK_STICK_Y] = nc->js.y;
        printf("nunchuk joystick calibration (min, center, max): x: %i, %i, %i  y: %i, %i, %i\n",
               nc->js.min.x,
               nc->js.center.x,
               nc->js.max.x,
               nc->js.min.y,
               nc->js.center.y,
               nc->js.max.y);
    } else if (wm->exp.type == EXP_CLASSIC) {
        /* classic controller */
        struct classic_ctrl_t* cc = (classic_ctrl_t*)&wm->exp.classic;

        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_ZL)) {
            printf("Classic: ZL pressed\n");
            joy_msg.buttons[CLASSIC_L] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_B)) {
            printf("Classic: B pressed\n");
            joy_msg.buttons[CLASSIC_B] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_Y)) {
            printf("Classic: Y pressed\n");
            joy_msg.buttons[CLASSIC_Y] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_A)) {
            printf("Classic: A pressed\n");
            joy_msg.buttons[CLASSIC_A] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_X)) {
            printf("Classic: X pressed\n");
            joy_msg.buttons[CLASSIC_X] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_ZR)) {
            printf("Classic: ZR pressed\n");
            joy_msg.buttons[CLASSIC_R] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_LEFT)) {
            printf("Classic: LEFT pressed\n");
            joy_msg.axes[WIIMOTE_DPAD_X] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_UP)) {
            printf("Classic: UP pressed\n");
            joy_msg.axes[WIIMOTE_DPAD_Y] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_RIGHT)) {
            printf("Classic: RIGHT pressed\n");
            joy_msg.axes[WIIMOTE_DPAD_X] = -1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_DOWN)) {
            printf("Classic: DOWN pressed\n");
            joy_msg.axes[WIIMOTE_DPAD_Y] = -1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_FULL_L)) {
            printf("Classic: FULL L pressed\n");
            joy_msg.buttons[CLASSIC_LSTICK_PUSH] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_MINUS)) {
            printf("Classic: MINUS pressed\n");
            joy_msg.buttons[CLASSIC_MINUS] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_HOME)) {
            printf("Classic: HOME pressed\n");
            joy_msg.buttons[CLASSIC_HOME] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_PLUS)) {
            printf("Classic: PLUS pressed\n");
            joy_msg.buttons[CLASSIC_PLUS] = 1;
        }
        if (IS_PRESSED(cc, CLASSIC_CTRL_BUTTON_FULL_R)) {
            printf("Classic: FULL R pressed\n");
            joy_msg.buttons[CLASSIC_RSTICK_PUSH] = 1;
        }

        printf("classic L button pressed:         %f\n", cc->l_shoulder);
        printf("classic R button pressed:         %f\n", cc->r_shoulder);
        printf("classic left joystick angle:      %f\n", cc->ljs.ang);
        printf("classic left joystick magnitude:  %f\n", cc->ljs.mag);
        printf("classic right joystick angle:     %f\n", cc->rjs.ang);
        printf("classic right joystick magnitude: %f\n", cc->rjs.mag);
    } else if (wm->exp.type == EXP_GUITAR_HERO_3) {
        /* guitar hero 3 guitar */
        struct guitar_hero_3_t* gh3 = (guitar_hero_3_t*)&wm->exp.gh3;

        if (IS_PRESSED(gh3, GUITAR_HERO_3_BUTTON_STRUM_UP)) {
            printf("Guitar: Strum Up pressed\n");
        }
        if (IS_PRESSED(gh3, GUITAR_HERO_3_BUTTON_STRUM_DOWN)) {
            printf("Guitar: Strum Down pressed\n");
        }
        if (IS_PRESSED(gh3, GUITAR_HERO_3_BUTTON_YELLOW)) {
            printf("Guitar: Yellow pressed\n");
        }
        if (IS_PRESSED(gh3, GUITAR_HERO_3_BUTTON_GREEN)) {
            printf("Guitar: Green pressed\n");
        }
        if (IS_PRESSED(gh3, GUITAR_HERO_3_BUTTON_BLUE)) {
            printf("Guitar: Blue pressed\n");
        }
        if (IS_PRESSED(gh3, GUITAR_HERO_3_BUTTON_RED)) {
            printf("Guitar: Red pressed\n");
        }
        if (IS_PRESSED(gh3, GUITAR_HERO_3_BUTTON_ORANGE)) {
            printf("Guitar: Orange pressed\n");
        }
        if (IS_PRESSED(gh3, GUITAR_HERO_3_BUTTON_PLUS)) {
            printf("Guitar: Plus pressed\n");
        }
        if (IS_PRESSED(gh3, GUITAR_HERO_3_BUTTON_MINUS)) {
            printf("Guitar: Minus pressed\n");
        }

        printf("Guitar whammy bar:          %f\n", gh3->whammy_bar);
        printf("Guitar joystick angle:      %f\n", gh3->js.ang);
        printf("Guitar joystick magnitude:  %f\n", gh3->js.mag);
    } else if (wm->exp.type == EXP_WII_BOARD) {
        /* wii balance board */
        struct wii_board_t* wb = (wii_board_t*)&wm->exp.wb;
        float total = wb->tl + wb->tr + wb->bl + wb->br;
        float x = ((wb->tr + wb->br) / total) * 2 - 1;
        float y = ((wb->tl + wb->tr) / total) * 2 - 1;
        printf("Weight: %f kg @ (%f, %f)\n", total, x, y);
        printf("Interpolated weight: TL:%f  TR:%f  BL:%f  BR:%f\n", wb->tl, wb->tr, wb->bl, wb->br);
        printf("Raw: TL:%d  TR:%d  BL:%d  BR:%d\n", wb->rtl, wb->rtr, wb->rbl, wb->rbr);
    }

    if (wm->exp.type == EXP_MOTION_PLUS ||
        wm->exp.type == EXP_MOTION_PLUS_NUNCHUK) {
        printf("Motion+ angular rates (deg/sec): pitch:%03.2f roll:%03.2f yaw:%03.2f\n",
               wm->exp.mp.angle_rate_gyro.pitch,
               wm->exp.mp.angle_rate_gyro.roll,
               wm->exp.mp.angle_rate_gyro.yaw);
    }
}

/**
 *	@brief Callback that handles a read event.
 *
 *	@param wm		Pointer to a wiimote_t structure.
 *	@param data		Pointer to the filled data block.
 *	@param len		Length in bytes of the data block.
 *
 *	This function is called automatically by the wiiuse library when
 *	the wiimote has returned the full data requested by a previous
 *	call to wiiuse_read_data().
 *
 *	You can read data on the wiimote, such as Mii data, if
 *	you know the offset address and the length.
 *
 *	The a data pointer was specified on the call to wiiuse_read_data().
 *	At the time of this function being called, it is not safe to deallocate
 *	this buffer.
 */
void handle_read(struct wiimote_t* wm, byte* data, unsigned short len) {
    int i = 0;

    printf("\n\n--- DATA READ [wiimote id %i] ---\n", wm->unid);
    printf("finished read of size %i\n", len);
    for (; i < len; ++i) {
        if (!(i % 16)) {
            printf("\n");
        }
        printf("%x ", data[i]);
    }
    printf("\n\n");
}

/**
 *	@brief Callback that handles a controller status event.
 *
 *	@param wm				Pointer to a wiimote_t structure.
 *	@param attachment		Is there an attachment? (1 for yes, 0 for no)
 *	@param speaker			Is the speaker enabled? (1 for yes, 0 for no)
 *	@param ir				Is the IR support enabled? (1 for yes, 0 for no)
 *	@param led				What LEDs are lit.
 *	@param battery_level	Battery level, between 0.0 (0%) and 1.0 (100%).
 *
 *	This occurs when either the controller status changed
 *	or the controller status was requested explicitly by
 *	wiiuse_status().
 *
 *	One reason the status can change is if the nunchuk was
 *	inserted or removed from the expansion port.
 */
void handle_ctrl_status(struct wiimote_t* wm) {
    printf("\n\n--- CONTROLLER STATUS [wiimote id %i] ---\n", wm->unid);

    printf("attachment:      %i\n", wm->exp.type);
    printf("speaker:         %i\n", WIIUSE_USING_SPEAKER(wm));
    printf("ir:              %i\n", WIIUSE_USING_IR(wm));
    printf("leds:            %i %i %i %i\n", WIIUSE_IS_LED_SET(wm, 1), WIIUSE_IS_LED_SET(wm, 2), WIIUSE_IS_LED_SET(wm, 3), WIIUSE_IS_LED_SET(wm, 4));
    printf("battery:         %f %%\n", wm->battery_level);
}

/**
 *	@brief Callback that handles a disconnection event.
 *
 *	@param wm				Pointer to a wiimote_t structure.
 *
 *	This can happen if the POWER button is pressed, or
 *	if the connection is interrupted.
 */
void handle_disconnect(wiimote* wm) {
    printf("\n\n--- DISCONNECTED [wiimote id %i] ---\n", wm->unid);
}

short any_wiimote_connected(wiimote** wm, int wiimotes) {
    int i;
    if (!wm) {
        return 0;
    }

    for (i = 0; i < wiimotes; i++) {
        if (wm[i] && WIIMOTE_IS_CONNECTED(wm[i])) {
            return 1;
        }
    }

    return 0;
}

int main(int argc, char* argv[]) {
    using namespace std::chrono_literals;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("joy_node");

    auto publisher = node->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

    std_msgs::msg::String greeting;
    greeting.data = "hello world";

    rclcpp::WallRate rate(1ms);
    wiimote** wiimotes;
    int found, connected;
    wiimotes = wiiuse_init(MAX_WIIMOTES);
    found = wiiuse_find(wiimotes, MAX_WIIMOTES, 5);
    if (!found) {
        printf("No wiimotes found.\n");
        return 0;
    }
    connected = wiiuse_connect(wiimotes, MAX_WIIMOTES);
    if (connected) {
        printf("Connected to %i wiimotes (of %i found).\n", connected, found);
    } else {
        printf("Failed to connect to any wiimote.\n");
        return 0;
    }
    wiiuse_set_leds(wiimotes[0], WIIMOTE_LED_1);
    wiiuse_rumble(wiimotes[0], 1);
    usleep(200000);
    wiiuse_rumble(wiimotes[0], 0);

    while (rclcpp::ok()) {
        if (wiiuse_poll(wiimotes, MAX_WIIMOTES)) {
            joy_msg.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            joy_msg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            /*
             *	This happens if something happened on any wiimote.
             *	So go through each one and check if anything happened.
             */
            int i = 0;
            for (; i < MAX_WIIMOTES; ++i) {
                switch (wiimotes[i]->event) {
                    case WIIUSE_EVENT:
                        /* a generic event occurred */
                        handle_event(wiimotes[i]);
                        break;

                    case WIIUSE_STATUS:
                        /* a status event occurred */
                        handle_ctrl_status(wiimotes[i]);
                        break;

                    case WIIUSE_DISCONNECT:
                    case WIIUSE_UNEXPECTED_DISCONNECT:
                        /* the wiimote disconnected */
                        handle_disconnect(wiimotes[i]);
                        break;

                    case WIIUSE_READ_DATA:
                        /*
                         *	Data we requested to read was returned.
                         *	Take a look at wiimotes[i]->read_req
                         *	for the data.
                         */
                        break;

                    case WIIUSE_NUNCHUK_INSERTED:
                        /*
                         *	a nunchuk was inserted
                         *	This is a good place to set any nunchuk specific
                         *	threshold values.  By default they are the same
                         *	as the wiimote.
                         */
                        /* wiiuse_set_nunchuk_orient_threshold((struct nunchuk_t*)&wiimotes[i]->exp.nunchuk, 90.0f); */
                        /* wiiuse_set_nunchuk_accel_threshold((struct nunchuk_t*)&wiimotes[i]->exp.nunchuk, 100); */
                        printf("Nunchuk inserted.\n");
                        break;

                    case WIIUSE_CLASSIC_CTRL_INSERTED:
                        printf("Classic controller inserted.\n");
                        break;

                    case WIIUSE_WII_BOARD_CTRL_INSERTED:
                        printf("Balance board controller inserted.\n");
                        break;

                    case WIIUSE_GUITAR_HERO_3_CTRL_INSERTED:
                        /* some expansion was inserted */
                        handle_ctrl_status(wiimotes[i]);
                        printf("Guitar Hero 3 controller inserted.\n");
                        break;

                    case WIIUSE_MOTION_PLUS_ACTIVATED:
                        printf("Motion+ was activated\n");
                        break;

                    case WIIUSE_NUNCHUK_REMOVED:
                    case WIIUSE_CLASSIC_CTRL_REMOVED:
                    case WIIUSE_GUITAR_HERO_3_CTRL_REMOVED:
                    case WIIUSE_WII_BOARD_CTRL_REMOVED:
                    case WIIUSE_MOTION_PLUS_REMOVED:
                        /* some expansion was removed */
                        handle_ctrl_status(wiimotes[i]);
                        printf("An expansion was removed.\n");
                        break;

                    default:
                        break;
                }
            }
        }

        // Set Joy Message Data
        joy_msg.header.stamp = node->get_clock()->now();
        joy_msg.header.frame_id = "joy";

        publisher->publish(joy_msg);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}