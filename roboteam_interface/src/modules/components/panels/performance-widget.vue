<script setup lang="ts">
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import RobotStpBadge from './stp-panel/robot-stp-badge.vue'
import { useSTPDataStore } from '../../stores/data-stores/stp-data-store'
import { useVisionDataStore } from '../../stores/data-stores/vision-data-store'
import { exec } from 'child_process';
import VueSocketIO from 'vue'
import Vue from 'vue';
import axios from 'axios';

const stpData = useSTPDataStore()
//const visionData = useVisionDataStore()

</script>

<template>
    <div style="margin-bottom: 10px;">
      <p v-if="is_match_started"></p>
      <p v-else>THE MATCH HAS NOT STARTED YET</p>
    </div>

    <p style="font-size: 20px; font-weight: bold;">Ratio Offensive/Defensive</p>

    <div style="margin-top: 10px;">
      <p v-if="is_match_started">Current play: {{stpData.latest?.currentPlay?.playName}}</p>
      <p v-else>Current play: None</p>
    </div>
    <div>
      <p>Number of passes tried: {{number_of_tried_passes}}</p>
    </div>
    <div>
      <p>Number of passes from kick tried: {{number_of_tried_passes_fkick}}</p>
    </div>
    <div>
      <p>Number of goalkeeper passes tried: {{number_of_tried_gk_passes}}</p>
    </div>
    <div>
      <p>Number of successful passes: {{number_of_successful_passes}}</p>
    </div>
    <div>
      <p v-if="is_match_started">Robot roles: {{robots_roles}}</p>
      <p v-else>List of roles: None</p>
    </div>
    <div>
      <p v-if="is_match_started">Passer ID: {{passer_id}}</p>
      <p v-else>Passer ID: None</p>
    </div>
    <div>
      <p v-if="is_match_started">Receiver ID: {{receiver_id}}</p>
      <p v-else>Receiver ID: None</p>
    </div>
    <div>
      <p v-if="is_match_started || (is_pass_tried || is_pass_fkick_tried || is_pass_gk_tried)">Receiver: ({{receiver_x}}, {{receiver_y}})</p>
      <p v-else>Receiver: None</p>
    </div>
    <div>
      <p v-if="is_match_started">Ratio attacking/defending: {{possession.toFixed(2)}} %</p>
      <p v-else>Ratio attacking/defending: None</p>
    </div>
    <div>
      <p v-if="is_match_started">Offensive counter: {{offensive_counter}}</p>
      <p v-else>Offensive counter: None</p>
    </div>
    <div>
      <p v-if="is_match_started">Defensive counter: {{defensive_counter}}</p>
      <p v-else>Defensive counter: None</p>
    </div>
    <div>
      <p v-if="is_match_started">Effective game time: {{(total_time/1000).toFixed(2)}} s</p>
      <p v-else>Effective game time: None</p>
    </div>

    <div>
      <p v-if="is_match_started">Counter of keeper actions: {{keeper_kick_counter}}</p>
      <p v-else>Counter of keeper actions: None</p>
    </div>

    <div>
      <p v-if="is_match_started">Average robots' speed: {{ instant_team_speed }}</p>
      <p v-else>Average robots' speed: None</p>
    </div>

    <div>
      <p style="font-size: 20px; font-weight: bold; margin-top: 10px;">HEATMAP:</p>
    </div>

    <div class="table-container">
    <table class="custom-table">
      <tbody>
        <tr v-for="(row, rowIndex) in x_y_positions_freq" :key="rowIndex">
          <td v-for="(cell, cellIndex) in row" :key="cellIndex">{{ cell }}</td>
        </tr>
      </tbody>
    </table>
    </div>

    <div class="centered">
      <button @click="send_to_server" class="my-boton">Download metrics</button>
    </div>

    <div class="centered">
      <p style="font-size: 20px; font-weight: bold; margin-top: 10px;">{{transmit_metrics}}</p>
    </div> 
    
    <div class="centered">
      <p style="font-size: 20px; font-weight: bold; margin-top: 10px;">{{data_sent_successfully}}</p>
    </div> 

</template>

<style scoped>

  .centered {
    display: flex;
    justify-content: center;
    align-items: center;
    margin-bottom: 20px;
  }
  .table-container {
  display: flex;
  justify-content: center;
  align-items: center;
  height: 100vh; 
  }
  .custom-table {
    border-collapse: collapse;
    width: 60%;
    height: 80%;
  }

  .custom-table th, .custom-table td {
    border: 1px solid #dddddd;
    text-align: left;
    padding: 8px;
  }

  .custom-table th {
    background-color: #f2f2f2;
  }

  .my-boton {
  background-color: #3498db; /* Color de fondo */
  color: #fff; /* Color del texto */
  padding: 10px 20px; /* Espaciado interno */
  border: none; /* Sin borde */
  border-radius: 5px; /* Bordes redondeados */
  cursor: pointer; /* Cambia el cursor al pasar por encima */
  font-size: 16px; /* Tamaño de fuente */
  transition: background-color 0.3s; /* Transición suave del color de fondo */
}

.my-boton:hover {
  background-color: #2980b9; /* Cambia el color de fondo al pasar por encima */
}
</style>

<script lang="ts">

import { defineComponent, ref, watch } from 'vue';
import { proto } from '../../../generated/proto';
//import Chart from '../../../../../node_modules/chart.js/auto/auto.mjs'
import Chart from 'chart.js/auto';
import STPRobot = proto.STPStatus.STPRobot
import TriState from '../ui-settings/tri-state.vue';

type RobotDictionary = { [key: string]: proto.STPStatus.ISTPRobot } | null;

export default defineComponent({
  data() {
    return {
      stpData: useSTPDataStore(),
      visionData: useVisionDataStore(),

      is_match_started: true, 
      is_kick_off: false, 
      is_there_passer: false, 
      is_there_passer_fkick: false,
      is_there_receiver: false,
      is_receiver_found: false, // This one is used in the loop through the list of robots to look for a receiver

      offensive_plays: ['Attack', 'Attacking Pass', 'Chipping Pass', 'Keeper Kick Ball', 'Free Kick Us Pass', 'Ball Placement Us Free Kick'],
      defensive_plays: ['Defend Shot', 'Defend Pass'],
      starting_plays: ['Kick Off Us', 'Kick Off Them'],
      previous_play: "None", 
      /*neutral_plays: ['Halt', 'BallPlacementUsFreeKick', 'BallPlacementThem', 'PenaltyThemPrepare', 'PenaltyUsPrepare', 'PenaltyThem', 'DefensiveStopFormation', 'AggressiveStopFormation',
      'PenaltyUs', 'KickOffUsPrepare', 'KickOffThemPrepare', 'FreeKickThem', 'FreeKickUsAtGoal', 'FreeKickUsPass', 'KickOffUs', 'KickOffThem'],*/
      offensive_counter: 0,
      defensive_counter: 0, 
      offensive_timer: 0,
      defensive_timer: 0, 
      //neutral_counter: 0,
      possession: 0, 

      current_play: 'Halt',
      last_time: 0, 
      interval_time: 0,
      total_time: 0,

      timer_switch: 0, 

      keeper_kick_counter: 0,
      robots_x_coord: [0,0,0,0,0,0,0,0,0,0,0],
      robots_y_coord: [0,0,0,0,0,0,0,0,0,0,0],
      robots_roles: ['None','None','None','None','None','None','None','None','None','None','None'],
      list_of_robots: null as RobotDictionary,
      x_y_positions_freq: [
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0]
      ],
      players_on_plus_6_field: 0,
      average_team_speed: 0, 
      instant_team_speed: 0, 
      concated_metrics: [],
      transmit_metrics : Array(6).fill(0),
      transmit_heatmap : Array(120).fill(0), 
      metrics : Array(126).fill(0),
      data_sent_successfully: false,
      aux_var: "variable type", 
      ball_x_coordinate: 0.0,
      ball_y_coordinate: 0.0,
      previous_passer_id: 0,
      previous_receiver_id: 0,
      passer_id: 0,
      receiver_id: 0, 
      passer_x: 0.0, 
      passer_y: 0.0,
      passer_x_fkick: 0.0, 
      passer_y_fkick: 0.0, 
      receiver_x: 0.0, 
      receiver_y: 0.0,
      is_pass_tried: false, 
      is_pass_fkick_tried: false, 
      is_pass_gk_tried: false,
      was_pass_already_successful: false, 
      is_pass_being_assessed: false, 
      receiver_changed_role: false,
      number_of_tried_passes: 0,
      number_of_tried_passes_fkick: 0, 
      number_of_tried_gk_passes: 0,
      number_of_successful_passes: 0, 

    };
  },
  beforeDestroy() {
    //this.downloadCSV();
  },
  methods: {
    possession_counters() {

      const playName = this.stpData.latest?.currentPlay?.playName;
      this.current_play = String(playName);
      
      if (this.defensive_plays.includes(String(playName))) {
        this.defensive_counter++;
        this.defensive_timer = this.defensive_timer + this.interval_time;
      } else if (this.offensive_plays.includes(String(playName))) {
        this.offensive_counter++;
        this.offensive_timer = this.offensive_timer + this.interval_time;
      } 

      this.possession = this.offensive_timer / (this.offensive_timer + this.defensive_timer) * 100

    },

    get_passer_position() { // Look for a passer and if there is one, return its position

      /*
      THE ROBOT ROLE IS IN THE STPDATA STORE, BUT THEIR POSITION IS IN VISION DATA...
      LET'S HOPE THAT THE POSITION IN BOTH STORES ARE THE SAME
      */

      /*
      Passer role is keep until receiver gets the pass or play is set to defensive, then it makes sense to 
      look at the position of the ball wrt the passer 
      */

      const playName = this.stpData.latest?.currentPlay?.playName;

      if ((this.defensive_plays.includes(String(playName)) || this.offensive_plays.includes(String(playName))) && playName !== 'Free Kick Us Pass') { 
        // Only look for passers when we are attacking

        let list_of_robots = this.robots_roles
        let robots_on_vision = this.visionData.ourRobots || []
        let counter = 0; // Let's assume that the position of each robot on the list list_of_robots is always the same
        // And use the counter to update the role of each robot

        for (let key in list_of_robots ) { // Go through the IDs of the robots

          //if (Object.prototype.hasOwnProperty.call(list_of_robots, key)) {

            //this.robots_roles[counter] = list_of_robots[key].role?.name!;

            if (list_of_robots[counter] === "passer") { //keeper_kick_ball -> PLAY & free_kick_taker -> ROLE

              this.is_there_passer = true; 
              const passer_id = counter; 

              // Get the coordinates of the passer
              this.passer_x = robots_on_vision[passer_id].pos?.x!;
              this.passer_y = robots_on_vision[passer_id].pos?.y!;

              this.passer_id = passer_id;
              return

            } else {

              this.is_there_passer = false; 
              this.passer_x = 0;
              this.passer_y = 0;

            }

            counter = counter + 1;

          //}
        }  
      } else {
        this.is_there_passer = false; 
        return
      }
    },

    get_passer_position_fkick() { // Look for a passer and if there is one, return its position

    const playName = this.stpData.latest?.currentPlay?.playName;

    if (this.defensive_plays.includes(String(playName)) || this.offensive_plays.includes(String(playName))) { // Only look for passers when we are attacking (have the)

      let list_of_robots = this.robots_roles
      let robots_on_vision = this.visionData.ourRobots || []
      let counter = 0; // Let's assume that the position of each robot on the list list_of_robots is always the same
      // And use the counter to update the role of each robot

      for (let key in list_of_robots ) { // Go through the IDs of the robots

        //if (Object.prototype.hasOwnProperty.call(list_of_robots, key)) {

          //this.robots_roles[counter] = list_of_robots[key].role?.name!;

          if (list_of_robots[key] === "free_kick_taker") { //The logic thing is free_kick_taker, but i have seen that the passer takes the role harasser

            this.is_there_passer_fkick = true; 
            const passer_id = counter; 

            /*if (passer_id === this.passer_id) { // If receiver is the same as in the previous iter, put his coordinates to zero
              this.is_there_passer_fkick = false; 
              this.passer_x_fkick = 0;
              this.passer_y_fkick = 0;
            }*/

            // Get the coordinates of the passer
            this.passer_x_fkick = robots_on_vision[passer_id].pos?.x!;
            this.passer_y_fkick = robots_on_vision[passer_id].pos?.y!;

            this.passer_id = passer_id;
            return

          } else {

            this.is_there_passer_fkick = false; 
            this.passer_x_fkick = 0;
            this.passer_y_fkick = 0;

          }
          counter = counter + 1;
        //}
      }  
    } else {
      this.is_there_passer = false; 
      return
    }
    },

    get_ball_position() {
      const world = this.visionData.latestWorld; 
      this.ball_x_coordinate = world?.ball?.pos!.x!; 
      this.ball_y_coordinate = world?.ball?.pos!.y!;
    },

    check_is_pass_tried() {
      if (this.is_there_receiver) { // If there is a passer check if he executes the pass (not if the pass arrives to the passer)
        //const ball_passer_distance = Math.sqrt((this.ball_x_coordinate-this.passer_x) ** 2 + (this.ball_y_coordinate-this.passer_y) ** 2); 
        this.is_pass_tried = true; 
        this.number_of_tried_passes = this.number_of_tried_passes + 1;
      }
    },

    check_is_pass_fkick_tried() {
      this.is_pass_fkick_tried = true; 
      this.number_of_tried_passes_fkick = this.number_of_tried_passes_fkick + 1;
    },

    check_is_gk_pass_tried() { // Directly check the plays and if the current one is Keeper Kick Ball, assume that a pass has been tried
      const playName = this.stpData.latest?.currentPlay?.playName;
      if (playName === 'Keeper Kick Ball' && playName !== this.previous_play) { 
          this.is_pass_gk_tried = true; 
          this.number_of_tried_gk_passes = this.number_of_tried_gk_passes + 1;
      }
      if (playName !== null && playName !== undefined) {
          this.previous_play = playName;
      }
    },

    look_4_receiver() {

      // First look for a passer_receiver or a harasser 
      // When this function is executed we already know there is a passer, look for him
      // (The booleans about whether a pass has been tried are in the "main" loop (function))

      let list_of_robots = this.robots_roles
      let robots_on_vision = this.visionData.ourRobots || []
      let counter = 0;
      this.is_receiver_found = false; 

      for (let key in list_of_robots ) { // Go through the IDs of the robots

        //if (Object.prototype.hasOwnProperty.call(list_of_robots, key)) {

          //this.robots_roles[counter] = list_of_robots[key].role?.name!; // Now don't update the list of robots (already done in get_passer_position())

          if (list_of_robots[counter] === "receiver") { // || list_of_robots[key].role?.name! === "harasser") { 

            // No need to ask whether there's a receiver, we assume there is one (whether pass_receiver or harasser)
            const receiver_id = counter; 

            // Get the coordinates of the passer
            this.receiver_x = robots_on_vision[receiver_id].pos?.x!;
            this.receiver_y = robots_on_vision[receiver_id].pos?.y!;

            this.receiver_id = receiver_id;
            this.is_receiver_found = true; 
            return

          } else {

            this.receiver_x = 0;
            this.receiver_y = 0;

          }
          counter = counter + 1;
        //}
      } 

    },

    check_is_still_receiver() { // Check if a robot (id) still has the role of receiver (this function is used during pass evaluation)

      let robot_id = this.receiver_id; // The robot we have to check whether it is still a receiver
      let list_of_robots = this.robots_roles
      let robots_on_vision = this.visionData.ourRobots || []
      //let counter = 0;


      //for (let key in list_of_robots ) { 

      //if (key === String(this.receiver_id) && Object.prototype.hasOwnProperty.call(list_of_robots, robot_id)) {

        // Update the roles list 
        //this.robots_roles[counter] = list_of_robots[key].role?.name!;
        //counter = counter + 1;

        //if (list_of_robots[robot_id].role?.name! === "receiver" || list_of_robots[robot_id].role?.name! === "passer" || list_of_robots[robot_id].role?.name! === "striker") {
        if (list_of_robots[robot_id] === "receiver" || list_of_robots[robot_id] === "passer" || list_of_robots[robot_id] === "striker") {
          // If the receiver is still a receiver, update its position
          this.receiver_changed_role = false; 
          this.receiver_x = robots_on_vision[this.receiver_id].pos?.x!;
          this.receiver_y = robots_on_vision[this.receiver_id].pos?.y!;

        } else {

          this.receiver_changed_role = true; 

        }
      //}

      //}

    },

    keeper_actions_counter() {
      if (this.current_play === 'Keeper Kick Ball') {
        this.keeper_kick_counter++;
      }
    },

    instant_position_calculator() {
      let robots = this.visionData.ourRobots || []
      let condition_plus_6 = []
      let x_coord_in_grid_sys = 0
      let y_coord_in_grid_sys = 0

      this.robots_x_coord = []
      this.robots_y_coord = []

        for (let robot of robots) {
          // Convertion from field coordinates to grid coordinates
          x_coord_in_grid_sys = Math.floor((Math.ceil(robot.pos?.x! + 6)+Math.floor(robot.pos?.x! + 6))/2) // Something .5
          y_coord_in_grid_sys = Math.floor((Math.ceil(robot.pos?.y! + 5)+Math.floor(robot.pos?.y! + 5))/2) // Something .5
          this.robots_x_coord.push(x_coord_in_grid_sys)
          this.robots_y_coord.push(y_coord_in_grid_sys)
        }
      condition_plus_6 = this.robots_x_coord.filter(item => item >= 5)
      this.players_on_plus_6_field = condition_plus_6.length

      // Add a number to the frequency on the position (index are the positions on the grid, values are the counts)
      // It does not necessarily has to add one each time, cause multiple robots can be in the same position of the grid...

      for (let i = 0; i < this.robots_x_coord.length; i++) { //Array of x and y coordinates have same coordinates
          this.x_y_positions_freq[this.robots_x_coord[i]][this.robots_y_coord[i]] += 1
        }
      
    },

    launch_py_server() {
      // netstat -tulpn | grep LISTEN
      const { spawn } = require('child_process')
      spawn('sh', ['run_pdf'], {
        cwd: '.'
      })
    },

    async send_to_server() {

      this.concat_metrics();

      try {
        const response = await fetch('http://127.0.0.1:50000/process_data', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            metrics: this.metrics,
          }),
        });

        if (response.ok) {
          const data = await response.json();
          console.log(data);
          this.data_sent_successfully = true; 
        } else {
          console.error('Error sending the message to the server');
        }
      } catch (error) {
        console.error('Network error', error);
      }
    },

    downloadCSV() {
      for (let i = 0; i < this.robots_x_coord.length; i++) { //Array of x and y coordinates have same coordinates
          this.x_y_positions_freq[this.robots_x_coord[i]][this.robots_y_coord[i]] += 1
        }

      const array2DJson = JSON.stringify(this.x_y_positions_freq);

    },

    to_csv(array: number[][]) {
      return array.map(row => row.join(',')).join('\n');
    },

    instant_speed_calculator() {
      let robots = this.visionData.ourRobots || []
      let vel_module = 0
  
      for (let robot of robots) {
        vel_module += Math.sqrt(Math.pow(robot.vel?.x!,2) + Math.pow(robot.vel?.y!,2)); 
      }

      this.instant_team_speed = vel_module/11 
    },

    concat_metrics() {

      let concated_metrics = [parseFloat((this.total_time/1000).toFixed(2)), 
                              parseFloat(this.possession.toFixed(2)), 
                              this.average_team_speed,
                              this.offensive_counter, 
                              this.defensive_counter, 
                              this.keeper_actions_counter]
      let flat_positions = this.x_y_positions_freq.flat()
      let metrics = [concated_metrics, flat_positions]

      this.transmit_metrics = concated_metrics
      this.transmit_heatmap = flat_positions
      this.metrics = metrics

    },

    define_event() {
      setInterval(() => {
        const playName = this.stpData.latest?.currentPlay?.playName;
        this.current_play = String(playName);

        if ((this.defensive_plays.includes(String(playName)) || this.offensive_plays.includes(String(playName))) && this.is_match_started === true) {

        this.average_team_speed = (this.average_team_speed*this.timer_switch + this.instant_team_speed*0.1) / (this.timer_switch + 0.1)
        this.timer_switch = this.timer_switch + 0.1;
        this.instant_speed_calculator();
        this.instant_position_calculator();
        }

        //this.get_ball_position();
      }, 500);
      
    }, 

    update_roles() {

      setInterval(() => {

      let list_of_robots = this.stpData.latest?.robots
      let counter = 0; 

      for (let key in list_of_robots ) { // Go through the IDs of the robots

        if (Object.prototype.hasOwnProperty.call(list_of_robots, key)) {

          this.robots_roles[counter] = list_of_robots[key].role?.name!;
          counter = counter + 1;

        }
      }
      }, 50);
    },

    passing_accuracy_block() {

      let is_pass_1_being_assessed = false;
      let is_pass_2_being_assessed = false;

      setInterval(() => {

        if (this.is_pass_being_assessed === false) { // If no pass is being processed to determine if it is successful, wait (look) for a passer

          this.get_passer_position(); // Here we also check if there is actually a passer (the boolean is_there_passer is set to true)
          
          // Look for a passer
          if (this.is_match_started && this.is_there_passer && this.previous_passer_id !== this.passer_id) { 

            // Only if there is a passer (and it is not the same as the previous pass), check if the pass has been tried
            this.get_ball_position();

            this.look_4_receiver() // Assume that there is going to be a receiver -> get its position

            if (this.is_receiver_found) {
              this.is_there_receiver = true;
            } else {
              this.is_there_receiver = false;
            }

            this.check_is_pass_tried(); // This function acknowledges that a pass has been tried if there is a receiver

            if (this.is_pass_tried) {
              is_pass_1_being_assessed = true;
              this.previous_passer_id = this.passer_id;
            }

          } else {

            is_pass_1_being_assessed = false;
            this.is_pass_tried = false; 

          }

          this.get_passer_position_fkick(); 

          // Look for a passer from free kick
          if (this.is_match_started && this.is_there_passer_fkick && this.previous_passer_id !== this.passer_id) { // Only if there is a passer, check if the pass has been tried
            
            this.get_ball_position();

            this.look_4_receiver(); // Assume that there is going to be a receiver -> get its position

            if (this.is_receiver_found) {
              this.is_there_receiver = true;
            } else {
              this.is_there_receiver = false;
            }

            this.check_is_pass_fkick_tried(); // This function acknowledges that a pass has been tried if there is a receiver

            if (this.is_pass_fkick_tried) {
              is_pass_2_being_assessed = true;
              this.previous_passer_id = this.passer_id;
            }

          } else {

            is_pass_2_being_assessed = false; 
            this.is_pass_fkick_tried = false;

          }

          // Determine if any of both passes must be assessed
          if (is_pass_1_being_assessed || is_pass_2_being_assessed) {
            this.is_pass_being_assessed = true;
          }

        } 
        
        else { // If there is a pass being assessed, see if it is sucessful, 
                 //run this code to compare the position of the ball (updated) with the fixed position of the receiver

          if (this.offensive_plays.includes(String(this.current_play))) { // If we are still in an ofensive play (haven't lost the ball)

            this.get_ball_position(); // Update the ball position
            this.check_is_still_receiver(); // Check receiver is still a receiver, a passer or a striker

            if (this.receiver_changed_role === false) { // If the receiver is still the receiver
              // Compare it with the last known position of the receiver
              const ball_receiver_distance = Math.sqrt((this.ball_x_coordinate-this.receiver_x) ** 2 + (this.ball_y_coordinate-this.receiver_y) ** 2); 

              if (ball_receiver_distance < 0.2 && this.previous_receiver_id !== this.receiver_id) { 

                this.number_of_successful_passes = this.number_of_successful_passes + 1;
                this.previous_receiver_id = this.receiver_id;
                this.is_pass_being_assessed = false; // Pass has been sucessful, assessment finished, look for another pass.

              } else { // Distance is still high still, but we are offensive and the receiver is still the receiver, have to keep evaluating the pass

                this.previous_receiver_id = this.receiver_id;
                this.is_pass_being_assessed = true;

              }

            } else {
              this.is_pass_being_assessed = false; // The pass has not been sucessful because the receiver changed its role
            }

          } 
          
          else {

            this.is_pass_being_assessed = false; // The pass has not been sucessful because we changed to a defensive play

          }

        }
      }, 100);
      
    }, 

    is_started() {
      const is_started_interval = setInterval(() => {
        const playName = this.stpData.latest?.currentPlay?.playName;
        this.current_play = String(playName);
        if (this.starting_plays.includes(String(playName)))
        {
          this.is_kick_off = true
          //clearInterval(is_started_interval)
        }
        if (this.is_kick_off && !this.starting_plays.includes(String(playName)))
        /*
        If the kick off has been already used as a play and now the play is not kick-off, then the match has started!
        */
        {
          this.is_match_started = true
          clearInterval(is_started_interval)
        }
      },
        100);
    }, 

  },

  created() {
    this.define_event(); // Init when the script is run
    this.is_started();
    this.update_roles();
    this.passing_accuracy_block();
  },

  mounted() {
    //this.launch_py_server();
    this.possession_counters();
    this.keeper_actions_counter();
  },

  watch: {
    'stpData.latest.currentPlay.playName'() { 
      if (this.last_time === 0) {
        this.last_time = Date.now();
      } else {
        this.interval_time = Date.now() - this.last_time;

        if (this.defensive_plays.includes(this.current_play) || 
            this.offensive_plays.includes(this.current_play)) {
              this.total_time = this.total_time + this.interval_time; // ms!
        }

        this.last_time = Date.now();
      }
      this.possession_counters();
      this.keeper_actions_counter();
    }
  }
});

</script>