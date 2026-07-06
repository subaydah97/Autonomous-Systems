#ifndef auxiliary_commands_h
#define auxiliary_commands_h "Ember's mod"

#include <Arduino.h>

// Declare extern variables. Defined in the main after this file is included.
extern float latestX;
extern float latestY;
extern float latestZ;

extern float latestRot;

extern uint32_t forwardLeftTicks;
extern uint32_t forwardRightTicks;

// Module variables

#define command_string_length 256
#define command_tokens 64

// Input variables
const char *delim = " ";
uint8_t tp = 0;

char command_string[command_string_length] = " ";
// Tokens for command_string
char *tokens[command_tokens];

extern bool emergency_flag; // Must be implemented in main.

struct auxiliary_command
{
    char *name;
    void (*func)(char *, char **, uint8_t &);
};

// Function declarations needed to build auxiliary commands structs
void override(char *command_string = command_string, char *tokens[command_tokens] = tokens, uint8_t &tp = tp);
void estop(char *command_string = command_string, char *tokens[command_tokens] = tokens, uint8_t &tp = tp);

auxiliary_command auxiliary_commands[] = {
    {"OVERRIDE", &override},
    {"ESTOP", &estop}};

//--------------------
// Utils
//--------------------
//  function used to read a float from *tokens[]
auto read_float(bool doIncrement = true)
{
    float returnVal = strtof(tokens[tp], &tokens[tp + 1]);

    if (doIncrement)
        tp++;

    return returnVal;
};

auto read_int(bool doIncrement = true)
{
    float returnVal = strtoimax(tokens[tp++], &tokens[tp], 10);

    if (doIncrement)
        tp++;

    return returnVal;
};

// -------------------------
// Functions
// -------------------------

void tokenize_input(char *command_string = command_string, char *tokens[command_tokens] = tokens, uint8_t &tp = tp)
{
    // Reset token pointer
    tp = 0;

    //
    char *token = strtok(command_string, delim);
    while (token != NULL)
    {
        tokens[tp++] = token;
        token = strtok(NULL, delim);
    }

    // Reset token pointer
    tp = 0;
}

// Should be called when the BOT/<id>/COMMANDS topic gets a message starting with OVERRIDE POSSITION
// Example input "OVERRIDE X 0.4 Y 0.9"
void override(char *command_string, char *tokens[16], uint8_t &tp)
{

    bool valid_token = true;

    // Loop through all tokens in the override
    while (valid_token)
    {
        char *token = tokens[tp++];

        switch (token[0])
        {
        case 'X':
        {
            float newX = read_float();
            latestX = newX;
        }
        break;

        case 'Y':
        {
            float newY = read_float();
            latestY = newY;
        }
        break;

        case 'Z':
        {
            float newZ = read_float();
            latestZ = newZ;
        }
        break;

        // Rotation
        case 'R':
        {
            float newRot = read_float();
            latestRot = newRot;
        }
        break;

        // Ticks
        case 'T': // Update both ticks
        case 'l': // Update only left
        case 'r': // update only right
        {
            float newTicks = read_int();
            if (token[0] == 'l')
                forwardLeftTicks = newTicks;
            if (token[0] == 'r')
                forwardRightTicks = newTicks;
            else
            {
                forwardLeftTicks = newTicks;
                forwardRightTicks = newTicks;
            }
        }
        break;

        // Unvalid token
        default:
            valid_token = false;
            break;
        }
    }
}

void estop(char *command_string, char *tokens[16], uint8_t &tp)
{
    emergency_flag = true;
}

// Routines

// Outline, must be finished
void mqtt_message_received(char *command_string = command_string, char *tokens[command_tokens] = tokens, uint8_t &tp = tp)
{
    // Tokennize input
    tokenize_input();

    // Potentialy to loop until a token reads NULL

    for (auxiliary_command command : auxiliary_commands)
    {
        if (strcasecmp(command.name, tokens[0]) == 0)
        {
            // Execute commands function
            command.func(command_string, tokens, tp);
        }
    }
}

#endif