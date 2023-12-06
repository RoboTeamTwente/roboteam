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

</template>

<script lang="ts">

import { defineComponent, ref, watch } from 'vue';
import { proto } from '../../../generated/proto';

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
      average_team_speed: 0, 
      instant_team_speed: 0
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
    }

  },

  created() {
    this.define_event(); // Init when the script is run
    this.is_started();
  },

  mounted() {
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