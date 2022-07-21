run_and_kill_python() {


python3 failure_simulation_with_random_traj.py &
python_pid=$!

sleep_time=30
sleep $sleep_time

kill $python_pid
}

repeats=5

for ((i=1;i<=$repeats;i++))
do
	echo "Iteration $i"
	run_and_kill_python
	zcho "Iteration $i completed"
done



