# generate task lists
python3 ../standard_cpu_task/rs_dataflow.py 128

cp ../standard_cpu_task/*.txt ../cpu_task/
./build/Garnet_standalone/gem5.opt \
	configs/example/garnet_synth_traffic.py \
	--topology=Mesh_XY \
	--num-cpus=16 \
	--num-dirs=16 \
	--mesh-rows=4 \
	--network=garnet2.0 \
	--inj-vnet=2 \
    --link_width_bits=128