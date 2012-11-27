import csv,sys
import math
import operator

class Node:
    #Class containing individual location

	def __init__(self, identifier, coordinates, deadline):
	    self.id = identifier
	    self.coordinates = coordinates
	    self.deadline = deadline

class Nodelist:
	#Class containing unvisited locations
	def __init__(self, nodes):	
		self.nodes = nodes

	def set_ranks (self, method, path):
		#Sort nodes and assign ids by index
		if method == 'line':
			pass

		if method == 'edist':
			for i in range(len(self.nodes)):
				current = path.nodelist[len(path.nodelist)-1]
				self.nodes[i].distance = edist(self.nodes[i].coordinates, current.coordinates)
			self.nodes = sorted(self.nodes, key=operator.attrgetter('distance'))

		if method == 'time':
			self.nodes = sorted(self.nodes, key=operator.attrgetter('deadline'))

		for i in range(len(self.nodes)):
			self.nodes[i].rank = i


class Path():

	def __init__( self ):
		self.nodelist = []
		self.totaltime = 0

	def set_time ( self, path ):
		self.totaltime = 0
		for i in range(len( self.nodelist ) - 1):
<<<<<<< HEAD
			self.totaltime = self.totaltime + eDist( i,i+1 )
=======
			self.totaltime = self.totaltime + edist( self.nodelist[i].coordinates,self.nodelist[i+1].coordinates )
>>>>>>> 629b99dd949def1b448d9e33d65c41a6a37d53ba

	def get_time ( self ):
		return self.totaltime

	def add_node( self, node ):
		self.nodelist.append( node )
		self.set_time( self.nodelist )

	def checkit (self, nodelist):
		sofar = self.get_time()
		for node in nodelist.nodes:

			if node.deadline < sofar:
				print str(node.id) + str(") TOO FAR: ") + str(node.deadline) + " - " + str(sofar)
				return False
		print str(node.id) + str(") OK") + str(node.deadline) + " - " + str(sofar)
		return True


def import_data( filename ):
	nodelist = []
	i = 0
	with open( str(filename), 'rb') as f:
		reader = csv.reader(f, delimiter=' ' )
		try:
			
			for row in reader:
				
				if len(row) is 2:
					nodelist.append( Node(i, [float(row[0]),float(row[1])], 0) )
					i = i + 1
				elif len(row) is 3:
					nodelist.append( Node(i, [float(row[0]),float(row[1])], float(row[2])) )
					i = i + 1
				else:
					print "something's gone wrong buddy"
			return nodelist
		except csv.Error as e:
			sys.exit('file %s, line %d: %s' % (file, reader.line_num, e))

def edist(pt1, pt2):
	return math.sqrt( math.pow((pt1[1]-pt2[1]),2) + math.pow((pt1[0]-pt2[0]),2) )

def RBackTracker(path, nodelist,method):
	#print "Length" + str(len(nodelist.nodes))
	if len(nodelist.nodes) is 0:
		return path
	else:
		for i in range( len(nodelist.nodes)):
			temp = nodelist.nodes[i]
			path.add_node(nodelist.nodes.pop(i))
			if path.checkit(nodelist):
				nodelist.set_ranks(method, path)
				result = RBackTracker(path,nodelist,method)
				return result
			else:
				path.nodelist.pop(i)
				nodelist.nodes.insert(i,temp)
				print "<--"
		return False

###############################################

#Initialize and Import
path = Path()
node_list = import_data( 'scenario1.txt'  )
node_list = Nodelist( node_list )
path.add_node( node_list.nodes.pop(0) )
method = 'line'

final_path = RBackTracker(path,node_list,method)

# print "-------------------------"

for i in range(len(final_path.nodelist)):
	print final_path.nodelist[i].id
	print final_path.nodelist[i].coordinates
	print final_path.nodelist[i].deadline
	#print final_path.nodelist[i].rank
	print "-------------------------"






'''

Real Manly Functions:
	RBacktracker(Path, Nodelist)
		if goal_test:
			return path
		else:
			for every Node in Nodelist: (by rank)
				pop Node to Path
				result = RBacktracker(Path, Nodelist) 
				if result is good 
					return Path
				if result is bad
					put it back
			return False

Flow:
	Initialize and import
	Run the motherfucker
	print the path
'''
