cp /home/fanx/noc/gem5_gt/standard_cpu_task/*.txt /home/fanx/noc/gem5_gt/cpu_task/
./build/Garnet_standalone/gem5.opt \
	configs/example/garnet_synth_traffic.py \
	--topology=Mesh_XY \
	--num-cpus=16 \
	--num-dirs=16 \
	--mesh-rows=4 \
	--network=garnet2.0 \
	--inj-vnet=0
