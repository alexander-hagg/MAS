#!/usr/bin/env python
from ants import *
import operator

# HINT: use A* for a start - has been proven to be pretty good

# define a class with a do_turn method
# the Ants.run method will parse and update bot input
# it will also run the do_turn method for us


# Initial State
# Player(s)
# Action(s)
#   for all ants: move    
#   prohibiting: stepping on ants, stepping on food, stepping on water
# Result(s)
#   moved, moved and eaten, moved and attacking enemy ant, moved and attacking enemy hill
# Terminal Test
#   all enemy hills dead, all enemy ants dead (not a good objective!)
# Utility Function (payoff function, objective function)
#   




class AsBotAsItGets:
    def __init__(self):
        # define class level variables, will be remembered between turns
        self.compass_directions = ('s','e','w','n')
        self.current_compass_direction = 0
        pass
    
    # do_setup is run once at the start of the game
    # after the bot has received the game settings
    # the ants class is created and setup by the Ants.run method
    def do_setup(self, ants):
        # initialize data structures after learning the game settings
        self.hills = []
        self.unseen = []

        for row in xrange(ants.rows):
            for col in xrange(ants.cols):
                if ants.passable((row,col)):
                    self.unseen.append((row, col))

        self.map = self.unseen
        
    # do turn is run once per turn
    # the ants class has the game state and is updated by the Ants.run method
    # it also has several helper methods to use

    def do_turn(self, ants):
        # track all moves, prevent collisions
        orders = {}
        def do_move_direction(loc, direction):
            new_loc = ants.destination(loc, direction)
            if (ants.unoccupied(new_loc) and new_loc not in orders and ants.passable(new_loc)):
                ants.issue_order((loc, direction))
                orders[new_loc] = loc
                return True
            else:
                return False
        
        targets = {}
        def do_move_location(loc, dest):
            directions = ants.direction(loc, dest)
            for direction in directions:
                if do_move_direction(loc, direction):
                    targets[dest] = loc
                    return True
            return False

        my_ants = ants.my_ants()
        enemy_ants = ants.enemy_ants()
        my_hills = ants.my_hills()
        enemy_hills = ants.enemy_hills()
        food = ants.food()

        

        # save unseen areas
        for loc in self.unseen[:]:
            if ants.visible(loc):
                self.unseen.remove(loc)

        # get all near objects
        for ant in my_ants:
            near_food = []
            near_ants = []
            near_enemy_ants = []
            near_hills = []
            near_enemy_hills = []
            #local_unseen = []
            dist_range = xrange(-6,6)

            enemy_ants = dict(ants.enemy_ants())

            for x, y in [(x,y) for x in dist_range for y in dist_range]:
                cur_x,cur_y = ant[0]+x,ant[1]+y
                dist = ants.distance(ant, (cur_x,cur_y))
                
                if (cur_x,cur_y) in ants.food():
                    near_food.append((dist,cur_x,cur_y))

                if (cur_x,cur_y) in ants.my_ants():
                    near_ants.append((dist,cur_x,cur_y))

                if (cur_x,cur_y) in enemy_ants:
                    near_enemy_ants.append((dist,cur_x,cur_y))

                if (cur_x,cur_y) in ants.my_hills():
                    near_hills.append((dist,cur_x,cur_y))

                if (cur_x,cur_y) in ants.enemy_hills():
                    near_enemy_hills.append((dist,cur_x,cur_y))

            near_food.sort()
            near_ants.sort()
            near_enemy_ants.sort()
            near_hills.sort()
            near_enemy_hills.sort()
            #local_unseen.sort()
 

            # attack enemy hills
            if len(near_enemy_hills) > 0:
                #print "CHARGE HILL"
                for near_enemy_hill in near_enemy_hills:
                    if do_move_location(ant, (near_enemy_hill[1],near_enemy_hill[2])):
                        break

            # attack when not outnumbered 
            if ant not in orders and len(near_enemy_ants) > 0 and len(near_ants) > len(near_enemy_ants):
                #print "CHARGE"
                for near_enemy_ant in near_enemy_ants:
                    if do_move_location(ant, (near_enemy_ant[1],near_enemy_ant[2])):
                        break

            # run away when outnumbered but stay when own hill is near
            if ant not in orders and len(near_enemy_ants) > 0 and len(near_ants) < len(near_enemy_ants) and len(near_hills) < 1:
                #print "OUTNUMBERED"
                for near_enemy_ant in near_enemy_ants:
                    if do_move_location(ant, (-near_enemy_ant[1],-near_enemy_ant[2])):
                        break


            # stay around own hill long enough
            #if ant not in orders and len(near_hills) > 0 and len(near_ants) < 3 and len(my_ants) > 20 and ant not in near_hills:
            #    break


            # look for food
            if ant not in orders and len(near_food) > 0:
                #print "look for food"
                for near_piece_of_food in near_food:
                    if do_move_location(ant, (near_piece_of_food[1],near_piece_of_food[2])):
                        break

            # do not allow groups of more than 4
            # if ant not in orders and len(near_ants) > 4:
            #     print "move away from group"
            #     for near_ant in near_ants:
            #         if do_move_location(ant, (-near_ant[1],-near_ant[2])):
            #             break

            # move with group
            if ant not in orders and len(near_ants) < 5 and len(near_ants) > 0:
                #print "move with group"
                for near_ant in near_ants:
                    if near_ant in orders:
                        #if do_move_direction(ant, orders[near_ant]):
                        if do_move_location(ant, orders[near_ant]):
                            break


            #visit unseen areas
            # if ant not in orders and len(local_unseen) > 0:
            #     #print "visit dense unseen areas"
            #     if not do_move_location(ant, (local_unseen[0][1],local_unseen[0][2])): 
            #         #print "taking the other way"
            #         do_move_location(ant, (-local_unseen[0][1],local_unseen[0][2]))
            #     for unseen in local_unseen:
            #         if do_move_location(ant, (unseen[1],unseen[2])): 
            #             break     

            # move in general direction
            #TODO follow a wall
            if ant not in orders:
                for cc_dir in self.compass_directions:
                    if not do_move_direction(ant, self.compass_directions[self.current_compass_direction]):
                        self.current_compass_direction = operator.mod(self.current_compass_direction+3,4)
                    else:
                        break

            #unblock hills
            if ant not in orders:
                for hill_loc in ants.my_hills():
                    if hill_loc in ants.my_ants() and hill_loc not in orders.values():
                        for direction in ('s','e','w','n'):
                            if do_move_direction(hill_loc, direction):
                                break


if __name__ == '__main__':
    # psyco will speed up python a little, but is not needed
    try:
        import psyco
        psyco.full()

    except ImportError:
        pass
    
    try:
        # if run is passed a class with a do_turn method, it will do the work
        # this is not needed, in which case you will need to write your own
        # parsing function and your own game state class
        Ants.run(AsBotAsItGets())
    except KeyboardInterrupt:
        print('ctrl-c, leaving ...')
