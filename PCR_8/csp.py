import csv,sys
import math
import operator
import copy

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
			self.totaltime = self.totaltime + edist( self.nodelist[i].coordinates,self.nodelist[i+1].coordinates )


	def get_time ( self ):
		return self.totaltime

	def add_node( self, node ):
		self.nodelist.append( node )
		self.set_time( self.nodelist )

	def remove_node(self, node):
		self.nodelist.remove(node)
		self.set_time(self.nodelist)

	def checkit (self, nodelist):
		global evals
		evals = evals + 1
		sofar = self.get_time()
		for node in nodelist.nodes:

			if node.deadline < sofar:
				print str(node.rank) + str(") TOO FAR: ") + str(node.deadline) + " - " + str(sofar)
				return False
		print str(" -- OK -- ")
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

def RBackTracker(_path, _nodelist,method):
	global plength
	path = copy.deepcopy(_path)
	nodelist = copy.deepcopy(_nodelist)



	if len(nodelist.nodes) is 0:
		return path
	else:

		#Sort list by chosen method
		nodelist.set_ranks(method, path)

		#Check every node by rank, testing if it satisfies constraints
		for i in range( len(nodelist.nodes)):
			
			print "\nNode tried: " + str(nodelist.nodes[i].id) + "   Length of nodelist: " + str(len(nodelist.nodes))
			temp = copy.deepcopy(nodelist.nodes.pop(i))
			
			#Add node and see if it satisfies contraints
			path.add_node(temp)

			if path.checkit(nodelist):				
				#If constraints are satisfied, go deeper
				print "-->"
				plength = plength + 1
				for j in range(len(path.nodelist)):
					print path.nodelist[j].id,
				result = RBackTracker(path,nodelist,method)
				if result:
					return result

			#put node back in node list
			path.remove_node(temp)
			nodelist.nodes.insert(i,temp)

		#Return list and go back up a level	
		print "<--"
		for j in range(len(path.nodelist)):
			print path.nodelist[j].id,
		return False

###############################################

#Initialize and Import
path = Path()
node_list = import_data( 'scenario5.txt'  )
node_list = Nodelist( node_list )
path.add_node( node_list.nodes.pop(0) )
method = 'time'

evals = 0
plength = 0

final_path = RBackTracker(path,node_list,method)

print "Evaluations: " + str(evals)
print "Path Length: " + str(plength)

# print "-------------------------"

# for i in range(len(final_path.nodelist)):
# 	print final_path.nodelist[i].id
# 	print final_path.nodelist[i].coordinates
# 	print final_path.nodelist[i].deadline
# 	#print final_path.nodelist[i].rank
# 	print "-------------------------"
# print len(final_path.nodelist)