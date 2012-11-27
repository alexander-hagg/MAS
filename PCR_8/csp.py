import csv,sys

class Path():

	def __init__( self ):
		self.nodelist = ['bogus']
		self.totaltime = 0

	def get_time ( self, path ):
		for i in range(len( self.nodelist ) - 1):
			self.totaltime = self.totaltime #+ eDist( i,i+1 )

	def goal_test ( self, nodelist ):
		if len( nodelist ) is len( self.nodelist ):
			return True
		else:
			return False

	def add_node( self, n, nodelist ):
		if ( n < len( nodelist ) ):
			self.nodelist.append( nodelist[n] )
			self.get_time( self.nodelist )
		else:
			print "n not in nodelist"

	pass

def import_data( filename ):
	nodelist = []

	with open( str(filename), 'rb') as f:
		reader = csv.reader(f, delimiter=' ' )

		try:
			i = 0
			for row in reader:
				if len(row) is 2:
					nodelist.append( Node(i, [row[0],row[1]], 0) )
				elif len(row) is 3:
					nodelist.append( Node(i, [row[0],row[1]], deadline) )
				else:
					print "something's gone wrong buddy"
				return nodelist
		except csv.Error as e:
			sys.exit('file %s, line %d: %s' % (file, reader.line_num, e))


path = Path()
nodelist = import_data( 'scenario1.txt'  )
print path.nodelist
print "trallalalala"
print nodelist

