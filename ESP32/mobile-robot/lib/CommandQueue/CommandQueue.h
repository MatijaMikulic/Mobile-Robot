#pragma once
#include <Arduino.h>

enum class CommandType{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    NONE
};

enum class CommandState {
    IDLE,
    EXECUTING,
    COMPLETED
};

typedef struct{
    CommandType type;     // forward (0), backward (1), right (2), left (3)
    int distance;         // in cm, for forward and backward commands
    int angle;            // in degs, for left and rotate commands
    CommandState state;
}Command;

class CommandQueue{
    private:     
        static const size_t MaxCommands = 10;
        Command commands[MaxCommands];
        size_t headPointer;
        size_t tailPointer;
        size_t commandCount;
    
    public:
        CommandQueue():headPointer(0),tailPointer(0),commandCount(0)
        {         
        }
        bool isFull(){
            return commandCount == MaxCommands;
        }
        bool isEmpty(){
            return commandCount == 0;
        }
        void push(const Command& command){
            if(!isFull()){
                commands[tailPointer] = command;
                tailPointer = (tailPointer + 1) % MaxCommands;
                commandCount++;
            }
        }
        Command pop(){
            Command command;
            if(!isEmpty()){
                command = commands[headPointer];
                headPointer = (headPointer+1) % MaxCommands;
                commandCount--;
            }else{
                command.type = CommandType::NONE; 
                command.distance = -1;
                command.angle = -1;
                command.state = CommandState::COMPLETED;
            }
            return command;
        }
        size_t size(){
            return commandCount;
        }
};
