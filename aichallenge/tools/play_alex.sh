#!/usr/bin/env sh
./playgame.py --player_seed 13 --end_wait=0.25 --verbose --log_dir game_logs --turns 1000 --map_file maps/symmetric_random_walk/random_walk_08.map "$@"  "python ../AsBotAsItGets.py" "python sample_bots/python/HunterBot.py" "python sample_bots/python/HunterBot.py" "python sample_bots/python/GreedyBot.py" "python sample_bots/python/LeftyBot.py"


#./playgame.py --player_seed 42 --end_wait=0.25 --verbose --log_dir game_logs --turns 200 --map_file maps/multi_hill_maze/multi_maze_03.map "$@" "python sample_bots/python/HunterBot.py" "python sample_bots/python/LeftyBot.py" "python sample_bots/python/GreedyBot.py" "python ../AsBotAsItGets.py"
