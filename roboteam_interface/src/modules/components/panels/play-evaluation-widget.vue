<script setup lang="ts">
import { ref, watch } from 'vue';
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { useSTPDataStore } from '../../stores/data-stores/stp-data-store'

const props = defineProps(['playName']);

const stpData = useSTPDataStore();

// Track play scores using a map
const playScoreMap = ref<Record<string, { count: number; startTime: number }>>({});

// Watch for changes in the current play and update the score accordingly
watch(() => stpData.latest?.currentPlay?.playName, (newPlay, oldPlay) => {
  if (newPlay !== oldPlay) {
    // A new play has started
    const currentTime = performance.now(); // Use performance.now() for higher precision
    const playInfo = playScoreMap.value[newPlay] || { count: 0, startTime: currentTime };
    playInfo.count += 1;
    if (oldPlay) {
      // Only update time spent if a previous play was in progress
      playInfo.timeSpent = ((currentTime - playInfo.startTime) / 1000).toFixed(2); // Convert to seconds
    }
    playScoreMap.value[newPlay] = playInfo;
  }
});
</script>
<template>
  <div>
    Play Evaluation
    <div class="overflow-x-auto">
      <table class="table w-full">
        <thead>
          <tr>
            <th>Play name</th>
            <th>Counter</th>
            <th>Time Spent (seconds)</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="play in stpData.latest?.scoredPlays" :key="play.playName!">
            <th class="inline-flex items-center gap-1">
              <font-awesome-icon
                v-if="play.playName === stpData.latest?.currentPlay?.playName"
                icon="circle"
                class="text-success w-3 h-3"
              />
              {{ play.playName }}
            </th>
            <td>{{ playScoreMap[play.playName]?.count || 0 }}</td>
            <td>{{ playScoreMap[play.playName]?.timeSpent || 0 }}</td>
            <td>
              <progress class="progress w-56" :value="playScoreMap[play.playName]?.count || 0" max="255"></progress>
            </td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>