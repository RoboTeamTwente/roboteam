<script setup lang="ts">
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { useSTPDataStore } from '../../stores/data-stores/stp-data-store'

const stpData = useSTPDataStore()

/*</script>

<template>
    Performance
    <div>
      <font-awesome-icon icon="fa-triangle-exclamation" />
      Current play: 
      {{stpData.latest?.currentPlay?.playName}}
    </div>
    <div>
      <font-awesome-icon icon="fa-triangle-exclamation" />
      {{possession}} %
    </div>
</template>

<script lang="ts">

import { defineComponent, ref, watch } from 'vue';

export default defineComponent({
  data() {
    return {
      stpData: {
        latest: {
          currentPlay: {
            playName: 'Halt' // Supongamos que es A al inicio
            // Puedes inicializar playName con cualquier valor A, B, C o D
          }
        }
      },
      offensive_plays: ['Attack', 'AttackingPass', 'KeeperKickBall'],
      defensive_plays: ['DefendShot', 'DefendPass', 'DefensiveStopFormation', 'AggressiveStopFormation'],
      neutral_plays: ['Halt', 'BallPlacementUs', 'BallPlacementThem', 'PenaltyThemPrepare', 'PenaltyUsPrepare', 'PenaltyThem', 
      'PenaltyUs', 'KickOffUsPrepare', 'KickOffThemPrepare', 'FreeKickThem', 'FreeKickUsAtGoal', 'FreeKickUsPass', 'KickOffUs', 'KickOffThem'],
      offensive_counter: 0,
      defensive_counter: 0, 
      nuetral_counter: 0,
      possession: 0
    };
  },
  methods: {
    increment_counters() {
      const playName = this.stpData.latest.currentPlay.playName;
      
      if (this.offensive_plays.includes(playName)) {
        this.offensive_counter++;
      } else if (this.defensive_plays.includes(playName)) {
        this.defensive_counter++;
      } else if (this.neutral_plays.includes(playName)) {
        this.nuetral_counter++;
      } 

      this.possession = this.offensive_counter / (this.offensive_counter + this.defensive_counter) * 100
    }
  },
  mounted() {
    this.increment_counters();
  },
  watch: {
    'stpData.latest.currentPlay.playName'() {
      this.increment_counters();
    }
  }
});

</script>*/