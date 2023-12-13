<script setup lang="ts">
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { useSTPDataStore } from '../../stores/data-stores/stp-data-store'
import { useVisionDataStore } from '../../stores/data-stores/vision-data-store'

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
      Average average speed: {{ average_team_speed }}
    </div>
    <div>

      <p style="font-size: 20px; font-weight: bold; margin-top: 10px;">Heatmap:</p>
    </div>

    <div>
      Plotting counter: {{plotting_counter}}
      Plotting value: {{ plotting_value }}
    </div>

    <div>
      <canvas id="myChart" width="800" height="400"></canvas>
    </div>

</template>

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
      heatmapData: [],

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
      average_team_speed: 0, 
      instant_team_speed: 0,
      plotting_counter: 0,
      plotting_value: 0,

      data_to_plot: {
        datasets: [{
          label: 'Datos de ejemplo',
          data: [
            { x: 10, y: 2 },
            { x: 15, y: 1 },
            { x: 25, y: 1.5 },
            { x: 30, y: 0.5 },
            { x: 5, y: 0 }
          ],
          backgroundColor: 'rgba(54, 162, 235, 0.5)', // Color de los puntos
          borderColor: 'rgba(54, 162, 235, 1)', // Color del borde de los puntos
          borderWidth: 1
        }]
      }
    };
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

    update_data_to_plot() {
      // Simula un cambio de valor dinámico cada 3 segundos (solo para demostración)
      const interval_to_update_plot = setInterval(() => {
        /*if (this.plotting_counter !== 0) {
          scatterChart.destroy()
        }*/
        this.data_to_plot = {
          datasets: [{
            label: 'Datos de ejemplo',
            data: [
              { x: 10, y: this.average_team_speed },
              { x: 15, y: 1 },
              { x: 25, y: 1.5 },
              { x: 30, y: 0.5 },
              { x: 5, y: 0 }
            ],
            backgroundColor: 'rgba(54, 162, 235, 0.5)', // Color de los puntos
            borderColor: 'rgba(54, 162, 235, 1)', // Color del borde de los puntos
            borderWidth: 1
          }]
        }
        this.plotting_value = this.data_to_plot.datasets[0].data[0].y
        this.plotting_counter = this.plotting_counter + 1

        const options = {
          responsive: true,
          maintainAspectRatio: false,
          scales: {
            x: {
              type: 'linear',
              position: 'bottom'
            },
            y: {
              type: 'linear',
              position: 'left'
            }
          }
        };

        let scatterChart = new Chart("myChart", {
          type: 'bar',
          data: this.data_to_plot, 
          options: options // Ignore this error
        });

        //scatterChart.data = this.data_to_plot;
        //scatterChart.update()
  
      }, 1000); // Actualizar cada 3 segundos (solo para demostración)
    },

    generateHeatmap() {
      var ctx = document.getElementById('myChart')

      var data = this.data_to_plot

      const options = {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: {
            type: 'linear',
            position: 'bottom'
          },
          y: {
            type: 'linear',
            position: 'left'
          }
        }
      };

      const scatterChart = new Chart("myChart", {
        type: 'bar',
        data: this.data_to_plot, 
        options: options // Ignore this error
      });
    }

  },

  created() {
    this.define_event(); // Init when the script is run
    this.is_started();
    this.update_data_to_plot();
  },

  mounted() {
    this.possession_counters();
    this.keeper_actions_counter();
    //this.generateHeatmap();
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
      //this.generateHeatmap();
    }
  }
});

</script>