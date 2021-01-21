# generate task listsstardard_create_file.py
python3 ../standard_cpu_task/rs_dataflow.py 128
python3 ../standard_cpu_output/stardard_create_file.py

cp ../standard_cpu_task/*.txt ../cpu_task/
cp ../standard_cpu_output/*.txt ../cpu_output/
./build/Garnet_standalone/gem5.opt \
	configs/example/garnet_synth_traffic.py \
	--topology=Mesh_XY \
	--num-cpus=16 \
	--num-dirs=16 \
	--mesh-rows=4 \
	--network=garnet2.0 \
	--inj-vnet=2 \
    --link_width_bits=128
