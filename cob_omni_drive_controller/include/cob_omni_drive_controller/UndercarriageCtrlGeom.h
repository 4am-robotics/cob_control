/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering   
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_undercarriage_ctrl
 * Description:
 *                              
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *          
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: April 2010:
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef UndercarriageCtrlGeom_INCLUDEDEF_H
#define UndercarriageCtrlGeom_INCLUDEDEF_H


#include <vector>
#include <string>

class UndercarriageCtrlGeom
{
public:
    struct PlatformState{
        double dVelLongMMS;
        double dVelLatMMS;
        double dRotRobRadS;

        const double get_vel_x() { return dVelLongMMS / 1000.0; }
        const double get_vel_y() { return dVelLatMMS / 1000.0; }

        void set_vel_x(const double val) { dVelLongMMS  = val * 1000.0; }
        void set_vel_y(const double val) { dVelLatMMS = val * 1000.0; }
        
        PlatformState() : dVelLongMMS(0), dVelLatMMS(0),dRotRobRadS(0) {}
    };
    struct WheelState {
        double dVelGearDriveRadS;
        double dVelGearSteerRadS;
        double dAngGearSteerRad;
        WheelState() : dVelGearDriveRadS(0), dVelGearSteerRadS(0), dAngGearSteerRad(0) {}
    };
    struct WheelParams{
        /** Position of the Wheels' Steering Axis'
        *  in cartesian (X/Y) coordinates
        *  relative to robot coordinate System
        */
        double dWheelXPosMM;
        double dWheelYPosMM;
        
        double dWheelNeutralPos;
        
        /** Factor between steering motion and steering induced motion of drive wheels
            *  subtract from Drive-Wheel Vel to get effective Drive Velocity (Direct Kinematics)
            *  add to Drive-Wheel Vel (Inverse Kinematics) to account for coupling when commanding velos
            */
        double dFactorVel;

        double dRadiusWheelMM;
        double dDistSteerAxisToDriveWheelMM;

        double dMaxDriveRateRadpS;
        double dMaxSteerRateRadpS;
        
        /** ------- Position Controller Steer Wheels -------
        * Impedance-Ctrlr Prms
        *  -> model Stiffness via Spring-Damper-Modell
        *  -> only oriented at impedance-ctrl (no forces commanded)
        *  m_dSpring   Spring-constant (elasticity)
        *  m_dDamp             Damping coefficient (also prop. for Velocity Feedforward)
        *  m_dVirtM    Virtual Mass of Spring-Damper System
        *  m_dDPhiMax  maximum angular velocity (cut-off)
        *  m_dDDPhiMax maximum angular acceleration (cut-off)
        */
        double dSpring, dDamp, dVirtM, dDPhiMax, dDDPhiMax;
    };
private:

    struct WheelData{
        WheelParams params_;
        
        WheelState state_;

        double m_dAngGearSteerTargetRad; // choosen alternativ for steering angle
        double m_dVelGearDriveTargetRadS;

        /** Exact Position of the Wheels' itself
            *  in cartesian (X/Y) and polar (Dist/Ang) coordinates
            *  relative to robot coordinate System
            */
        double m_dExWheelXPosMM;
        double m_dExWheelYPosMM;
        double m_dExWheelDistMM;
        double m_dExWheelAngRad;

        double m_dVelWheelMMS;
        
        // previous Commanded deltaPhi e(k-1)
        // double m_dCtrlDeltaPhi; not used
        // previous Commanded Velocity u(k-1)
        double m_dCtrlVelCmdInt;

        // calculate inverse kinematics
        void setTarget(const PlatformState &state);

        void updateState(const WheelState &state);

        void calcControlStep(WheelState &command, double dCmdRateS, bool reset);

        static double mergeRotRobRadS(const WheelData &wheel1, const WheelData &wheel2);
        
        WheelData(const WheelParams &params) : params_(params), /*m_dCtrlDeltaPhi(0),*/  m_dCtrlVelCmdInt(0){
            updateState(WheelState());
            setTarget(PlatformState());
        }

        void reset();

    };

    std::vector<WheelData> wheels_;
        
public:
    // Constructor
    UndercarriageCtrlGeom(const std::vector<WheelParams> &params);

    static void parseIniFiles(std::vector<WheelParams> &params, const std::string &path);

    // Set desired value for Plattform Velocity to UndercarriageCtrl (Sollwertvorgabe)
    void setTarget(const PlatformState &state);

    // Get result of direct kinematics
    void calcDirect(PlatformState &state);

    // Set actual values of wheels (steer/drive velocity/position) (Istwerte)
    void updateWheelStates(const std::vector<WheelState> &states);

    // Get set point values for the Wheels (including controller) from UndercarriangeCtrl
    void calcControlStep(std::vector<WheelState> &states, double dCmdRateS, bool reset);

    void reset();


};
#endif
