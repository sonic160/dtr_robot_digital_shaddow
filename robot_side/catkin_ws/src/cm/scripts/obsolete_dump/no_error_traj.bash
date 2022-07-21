echo "NO FAILURE"

for failure_type in {3..3}
do
	for motor in {6..6}
	do
	echo "motor $motor ; failure_type $failure_type"
	python3 failure_simulation_with_chosen_failure.py $motor $failure_type &
python_pid=$!

	sleep_time=30
	sleep $sleep_time
	kill $python_pid
	zcho "COMPLETED motor $motor ; failure_type $failure_type"
	done
	
done
  
