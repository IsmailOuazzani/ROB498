# GameInfo.msg

std_msgs/Header header
string map_name

# Enum for game state definitions
uint8 GAME_STATE_WON        = 0
uint8 GAME_STATE_LOST       = 1
uint8 GAME_STATE_BLIND_INDEF= 2
uint8 GAME_STATE_BLIND_1    = 3
uint8 GAME_STATE_BLIND_2    = 4
uint8 GAME_STATE_BLIND_3    = 5
uint8 GAME_STATE_SEEKING    = 6
uint8 GAME_STATE_STOP       = 7

uint8 game_state



