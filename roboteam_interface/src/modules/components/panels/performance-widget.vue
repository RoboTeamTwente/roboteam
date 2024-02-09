<script setup lang="ts">
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { useSTPDataStore } from '../../stores/data-stores/stp-data-store'
import { useVisionDataStore } from '../../stores/data-stores/vision-data-store'
import { exec } from 'child_process';
import VueSocketIO from 'vue-socket.io'

const stpData = useSTPDataStore()
const visionData = useVisionDataStore()

</script>

<template>
    <p style="font-size: 20px; font-weight: bold;">Ratio Offensive/Defensive</p>
    <div style="margin-top: 10px;">
      Current play: 
      {{stpData.latest?.currentPlay?.playName}}
    </div>
    <div>
      Ratio:
      {{possession.toFixed(2)}} %
    </div>
    <div>
      Offensive counter:
      {{offensive_counter}}
    </div>
    <div>
      Defensive counter:
      {{defensive_counter}}
    </div>
    <div>
      Effective game time:
      {{(total_time/1000).toFixed(2)}}
    </div>

    <p style="font-size: 20px; font-weight: bold; margin-top: 10px;">Counter of keeper actions:</p>
    <div>
      {{keeper_kick_counter}}
    </div>

    <p style="font-size: 20px; font-weight: bold; margin-top: 10px;">Average robots' speed:</p>
    <div>
      Instant average speed: {{ instant_team_speed }}
    </div>
    <div>
      Average speed: {{ average_team_speed }}
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
      <button @click="downloadCSV" class="my-boton">Download metrics</button>
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
export default defineComponent({
  data() {
    return {
      stpData: useSTPDataStore(),
      visionData: useVisionDataStore(),

      match_started: false, 

      offensive_plays: ['Attack', 'AttackingPass', 'ChippingPass'],
      defensive_plays: ['Defend Shot', 'Defend Pass', 'Keeper Kick Ball'],
      /*neutral_plays: ['Halt', 'BallPlacementUs', 'BallPlacementThem', 'PenaltyThemPrepare', 'PenaltyUsPrepare', 'PenaltyThem', 'DefensiveStopFormation', 'AggressiveStopFormation',
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
      instant_team_speed: 0

    };
  },
  beforeDestroy() {
    this.downloadCSV();
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
        this.offensive_timer = this.offensive_timer + this.interval_time;;
      } 

      this.possession = this.offensive_timer / (this.offensive_timer + this.defensive_timer) * 100

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
      spawn('sh', ['run_pdf.sh'], {
        cwd: '.'
      })
    },

    downloadCSV() {
      for (let i = 0; i < this.robots_x_coord.length; i++) { //Array of x and y coordinates have same coordinates
          this.x_y_positions_freq[this.robots_x_coord[i]][this.robots_y_coord[i]] += 1
        }

      const array2DJson = JSON.stringify(this.x_y_positions_freq);
      //const command = `python3 heatmap_plotter.py "${array2DJson}"`;
      

      // Exec the secundary process
      /*const proceso = exec(command, (error, stdout, stderr) => {
        if (error) {
          console.error(`Error al ejecutar el script: ${error}`);
          return;
        }

        // Manejar cualquier error en la salida estándar de error
        if (stderr) {
          console.error(`Error en la salida estándar de error: ${stderr}`);
        }
      });*/
    
      const csv = this.to_csv(this.x_y_positions_freq);

      const blob = new Blob([csv], { type: 'text/csv' });
      const url = URL.createObjectURL(blob);

      const a = document.createElement('a');
      a.href = url;
      a.download = 'heatmap.csv';
      // Crea un enlace <a> para la descarga
      document.body.appendChild(a);
      a.click();

      document.body.removeChild(a);
      URL.revokeObjectURL(url);

      const { spawn } = require('child_process')
      spawn('sh', ['run_pdf.sh', csv], {
        cwd: '.'
      })

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

    define_event() {
      setInterval(() => {
        const playName = this.stpData.latest?.currentPlay?.playName;
        this.current_play = String(playName);

        if ((this.defensive_plays.includes(String(playName)) || this.offensive_plays.includes(String(playName))) && this.match_started === true) {

        this.average_team_speed = (this.average_team_speed*this.timer_switch + this.instant_team_speed*0.1) / (this.timer_switch + 0.1)
        this.timer_switch = this.timer_switch + 0.1;
        this.instant_speed_calculator();
        this.instant_position_calculator();
        }
      }, 100);
      
    }, 

    is_started() {
      const is_started_interval = setInterval(() => {
        const playName = this.stpData.latest?.currentPlay?.playName;
        this.current_play = String(playName);
        if (!this.defensive_plays.includes(String(playName)) && !this.offensive_plays.includes(String(playName)))
        {
          this.match_started = true
          clearInterval(is_started_interval)
        }},
        100);
    }, 

  },

  created() {
    this.define_event(); // Init when the script is run
    this.is_started();
  },

  mounted() {
    this.possession_counters();
    this.keeper_actions_counter();
    this.launch_py_server();
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