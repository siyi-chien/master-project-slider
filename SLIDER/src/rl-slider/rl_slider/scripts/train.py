#!/usr/bin/env python2
#/******************************************************************************
#-Author: Siyi
#-SLIDER project, Robot Intelligence Lab, Imperial College London, 2023
#******************************************************************************/

import gym
import gym_gazebo
import time
from distutils.dir_util import copy_tree
import os
import json
import liveplot
import deepq
import sys


sys.path.append('/home/siyi/SLIDER/src/slider_controller')
sys.path.append('/home/siyi/SLIDER/src/rl-slider/rl_slider/envs')
from rl_slider.envs import SliderEnv



def detect_monitor_files(training_dir):
    return [os.path.join(training_dir, f) for f in os.listdir(training_dir) if f.startswith('openaigym')]

def clear_monitor_files(training_dir):
    files = detect_monitor_files(training_dir)
    if len(files) == 0:
        return
    for file in files:
        print(file)
        os.unlink(file)

if __name__ == '__main__':

    env = gym.make('slider_env-v0')
    outdir = '/home/siyi/SLIDER/src/rl-slider/rl_slider/out/'
    path = '/home/siyi/SLIDER/src/rl-slider/rl_slider/out/'
    plotter = liveplot.LivePlot(outdir)

    continue_execution = False

    resume_path = path
    weights_path = resume_path + '.h5'
    monitor_path = resume_path
    params_json  = '/home/siyi/SLIDER/src/rl-slider/rl_slider/scripts/parameters.json'

    with open(params_json) as outfile:
        d = json.load(outfile)
        epochs = d.get('epochs')
        steps = d.get('steps')
        updateTargetNetwork = d.get('updateTargetNetwork')
        explorationRate = d.get('explorationRate')
        minibatch_size = d.get('minibatch_size')
        learnStart = d.get('learnStart')
        learningRate = d.get('learningRate')
        discountFactor = d.get('discountFactor')
        memorySize = d.get('memorySize')
        network_inputs = d.get('network_inputs')
        network_outputs = d.get('network_outputs')
        network_structure = d.get('network_structure')
        current_epoch = d.get('current_epoch')


    deepQ = deepq.DeepQ(network_inputs, network_outputs, memorySize, discountFactor, learningRate, learnStart)
    deepQ.initNetworks(network_structure)

    # deepQ.loadWeights('/home/siyi/SLIDER/src/rl-slider/rl_slider/out100.h5')
    clear_monitor_files(outdir)
    copy_tree(monitor_path,outdir)

    env._max_episode_steps = steps # env returns done after _max_episode_steps
    #env = gym.wrappers.Monitor(env, outdir,force=not continue_execution, resume=continue_execution)

    last100Scores = [0] * 100
    last100ScoresIndex = 0
    last100Filled = False
    stepCounter = 0
    highest_reward = 0

    start_time = time.time()
    #start iterating from 'current epoch'.
    for epoch in range(current_epoch+1, epochs+1, 1):
        print("Epoch: ", epoch)
        observation,info = env.reset()
        cumulated_reward = 0
        done = False
        episode_step = 0

        # run until env returns done
        while not done:
            # env.render()
            qValues = deepQ.getQValues(observation)
            action = deepQ.selectAction(qValues, explorationRate)
         
            newObservation, reward, done, info = env.step(action)

            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            deepQ.addMemory(observation, action, reward, newObservation, done)

            if stepCounter >= learnStart:
                if stepCounter <= updateTargetNetwork:
                    deepQ.learnOnMiniBatch(minibatch_size, False)
                else :
                    deepQ.learnOnMiniBatch(minibatch_size, True)

            observation = newObservation

            if done:
               
                last100Scores[last100ScoresIndex] = episode_step
                last100ScoresIndex += 1
                if last100ScoresIndex >= 100:
                    last100Filled = True
                    last100ScoresIndex = 0
                if not last100Filled:
                    print ("EP " + str(epoch) + " - " + format(episode_step + 1) + "/" + str(steps) + " Episode steps   Exploration=" + str(round(explorationRate, 2)))
                else :
                    m, s = divmod(int(time.time() - start_time), 60)
                    h, m = divmod(m, 60)
                    print ("EP " + str(epoch) + " - " + format(episode_step + 1) + "/" + str(steps) + " Episode steps - last100 Steps : " + str((sum(last100Scores) / len(last100Scores))) + " - Cumulated R: " + str(cumulated_reward) + "   Eps=" + str(round(explorationRate, 2)) + "     Time: %d:%02d:%02d" % (h, m, s))
                    if (epoch)%100==0:
                        #save simulation parameters.
                        parameter_keys = ['epochs','steps','updateTargetNetwork','explorationRate','minibatch_size','learnStart','learningRate','discountFactor','memorySize','network_inputs','network_outputs','network_structure','current_epoch']
                        parameter_values = [epochs, steps, updateTargetNetwork, explorationRate, minibatch_size, learnStart, learningRate, discountFactor, memorySize, network_inputs, network_outputs, network_structure, epoch]
                        parameter_dictionary = dict(zip(parameter_keys, parameter_values))


            stepCounter += 1
            if stepCounter % updateTargetNetwork == 0:
                deepQ.updateTargetNetwork()
                print ("updating target network")

            episode_step += 1

        explorationRate *= 0.995 #epsilon decay
        explorationRate = max (0.1, explorationRate)

        #if epoch % 10 == 0:
           # plotter.plot(env)

    env.close()
