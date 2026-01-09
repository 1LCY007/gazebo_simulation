/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>
#ifdef RUN_ROS
#include <ros/ros.h>
#endif

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
#ifdef COMPILE_WITH_MOVE_BASE
    _stateList.moveBase = new State_move_base(_ctrlComp);
#endif  // COMPILE_WITH_MOVE_BASE
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _autoStartTime = getSystemTime();
    _autoFixedStandTime = 1;
    _autoFixedStandDelayUs = 200000;
    _autoMoveBaseDelayUs = 5000000;
    bool auto_fixedstand = true;
    bool auto_move_base = true;
#ifdef RUN_ROS
    double auto_fixedstand_delay_s = 0.2;
    double auto_move_base_delay_s = 3.0;
    ros::param::param("~auto_fixedstand_delay_s", auto_fixedstand_delay_s, 0.2);
    ros::param::param("~auto_move_base_delay_s", auto_move_base_delay_s, 5.0);
    _autoMoveBaseDelayUs = static_cast<long long>(auto_move_base_delay_s * 1000000.0);
    _autoFixedStandDelayUs = static_cast<long long>(auto_fixedstand_delay_s * 1000000.0);
    ros::param::param("~auto_fixedstand", auto_fixedstand, false);
#ifdef COMPILE_WITH_MOVE_BASE
    ros::param::param("~auto_move_base", auto_move_base, false);
#endif
#endif
    if (auto_move_base) {
        auto_fixedstand = true;
    }
    _autoFixedStandSent = !auto_fixedstand;
    _autoMoveBaseSent = !auto_move_base;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run(){
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();

    if (_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO) {
        const long long now = getSystemTime();
        if (!_autoFixedStandSent && (now - _autoStartTime) >= _autoFixedStandDelayUs) {
            if (_ctrlComp->lowState->userCmd == UserCommand::NONE) {
                _ctrlComp->lowState->userCmd = UserCommand::L2_A;
            }
            _autoFixedStandSent = true;
            _autoFixedStandTime = now;
        }
#ifdef COMPILE_WITH_MOVE_BASE
        if (_autoFixedStandSent && !_autoMoveBaseSent &&
            (now - _autoFixedStandTime) >= _autoMoveBaseDelayUs) {
            if (_ctrlComp->lowState->userCmd == UserCommand::NONE) {
                _ctrlComp->lowState->userCmd = UserCommand::L2_Y;
            }
            _autoMoveBaseSent = true;
        }
#endif  // COMPILE_WITH_MOVE_BASE
    }

    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run();
    if(!checkSafty()){
        _ctrlComp->ioInter->setPassive();
    }

    if(_mode == FSMMode::NORMAL){
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
#ifdef COMPILE_WITH_MOVE_BASE
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}
