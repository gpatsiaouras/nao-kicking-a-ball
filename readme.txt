there are 2 implementations of the simulator
1 Evolution
2 Reinforcement Learning

They both use the same part of code about the simulation routine, with some light differences.

In which way they are similar:
	Open the world with Webots, it is located The World is located in ./Webots Base World/*.wbt
	
	The world will load the main code, located in controllers/God/God.py, and it is the code associated with the object "Robot" in the Webots' world, an object with the only scope to control the simulation.
	
	Will load even /controllers/Nao_Controller/Nao_Controller.py that is the controller of the Nao.
	
	Basics motion files are located in ./motions where is possible to find the base Shoot.motion used as starting base in the experiments.
	
	In /data will be saved at each iteration the file God.npy, that is the instance of the class God that is saved and loaded at each simulation(in format npy with pickle library)
	IMPORTANT: To restart from zero the simulation is it necessary to delete manually this file "God.npy", otherwise, wherever interrupted, the simulation will restart where it left.
	The population in test is saved in data/population
	
	Unfortunately Webots revealed itself to be a very bad simulator(or very difficult and unclear to set properly), farther the team of softbank didn't put much effort to set a clean base environment to work on. Even if the motion of the robot would exceed the limits, Webots would anyway simulate them triggering a warning. On the real Nao this motion would not even start.
	For this reason we had to implement the class "motion_util" present in God.py, that is in charge to convert a motion file that doesn't respect the limits in one that it does, and to correct properly in the evolution and learning process the values that exceed this limits.
		this limits are located in controllers/limits.txt and they are loaded one time, at the first init
	
		# to correct a file that is in the limits
		util = motion_util()
		util.open(r"../../motions/ind_37.motion")
		util.respect_limits()
		
		# only to check if it is in the limits: (after had it open as above)
		util.check_limits(False)
		
		# It is possible to add some time to the start of a motion file: (useful in the real tests)
		util.add_initial_time_to_motion_file(0.1)
	
	The code that start and run the simulation is in the bottom of God.py
		controller.run() # will start the simulation, after the initialization()
	Other function can be executed like:
		controller.god.plot_it() # plot the scores to check convergences
		controller.only_run(35) # only run child number 35
		controller.run_original() # run the original motion file
		controller.run_the_best() # run the best child # this could be not available in RL
	
Evolution:
	The evolution implement a different backup function, every tot generations.
	
Reinforcement Learning:
	The Reinforcement Learning approach born with an handCraftedKick.motion and then converted to be used with the base Shoot.motion to compare the results.
	
	There is two worlds, the nao_robocup_long.wbt is a difference start, more far from the door, that was the one initially used with handCraftedKick.motion
	
	Doesn't implement a backup function as evolution code does, but instead save each generation, if restarted, will overwrite all the generations, one by one. It doesn't backup the god.npy file