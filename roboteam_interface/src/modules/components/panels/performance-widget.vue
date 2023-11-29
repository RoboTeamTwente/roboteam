<script setup lang="ts">
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { useSTPDataStore } from '../../stores/data-stores/stp-data-store'

const stpData = useSTPDataStore()

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
</template>

<script lang="ts">

import { defineComponent, ref, watch } from 'vue';

export default defineComponent({
  data() {
    return {
      stpData: useSTPDataStore(),
      offensive_plays: ['Attack', 'AttackingPass', 'KeeperKickBall'],
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
      total_time: 0
    };
  },
  methods: {
    increment_counters() {

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

    }

  },
  mounted() {
    this.increment_counters();
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
      this.increment_counters();
    }
  }
});

</script>