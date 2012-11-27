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
		self.nodelist = ['bogus']
		self.totaltime = 0

	def set_time ( self, path ):
		for i in range(len( self.nodelist ) - 1):
			self.totaltime = self.totaltime #+ eDist( i,i+1 )

	def get_time ( self ):
		return self.totaltime

	def add_node( self, node ):
		self.nodelist.append( node )
		self.set_time( self.nodelist )

	pass

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
	return math.sqrt( (pt1[1]-pt2[1])**2 + (pt1[0]-pt2[0])**2 )



path = Path()
node_list = import_data( 'scenario1.txt'  )


node_list = Nodelist( node_list )

path.add_node( node_list.nodes.pop(0) )


node_list.set_ranks('edist', path)
print "-------------------------"

for i in range(len(node_list.nodes)):
	print node_list.nodes[i].id
	print node_list.nodes[i].coordinates
	print node_list.nodes[i].deadline
	print node_list.nodes[i].rank
	print "-------------------------"




'''
Data Structures:
	Path
		node[]
		get_Time (path)
		goal_Test (path)
		add_node(n, Nodelist)

X	Node
X		id
X		rank
X		coordinates
X		deadline

X	Nodelist
X		node[]
X		set_Ranks(path, method)
		
import_data()
XeDist()

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
