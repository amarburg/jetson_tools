#!/usr/bin/env ruby

require_relative 'lights'

begin

25.times.each { |i|
	p i*10
	$lights[0].send( i*10 )
	$lights[1].send( i*10 )
	sleep(1)
}

rescue Exception=>e
end

$lights.each { |l| l.off }
