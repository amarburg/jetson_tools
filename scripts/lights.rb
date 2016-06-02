

class Light

	def initialize( addr )
		@addr = addr
	end

	def off
		send( 0 )
	end

	def send( val )
		system 'i2cset', '-y', '0', "0x%02x" % @addr,  "0x%02x" % val
	end
end

$lights = [ Light.new( 0x12 ), Light.new( 0x13 ) ]
