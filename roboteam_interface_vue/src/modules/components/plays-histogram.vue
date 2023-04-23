<script setup lang="ts">
import {FontAwesomeIcon} from "@fortawesome/vue-fontawesome";
import {useAIStore} from "../stores/ai-store";

const aiStore = useAIStore()


</script>

<template>
  Play History
  <div v-if="aiStore.gameController.playHistory.length === 0" class="text-sm">No plays yet</div>
  <div class="text-sm breadcrumbs">
    <ul>
      <li v-for="play in aiStore.gameController.playHistory">{{play}}</li>
    </ul>
  </div>

  Play Evaluation
  <div class="overflow-x-auto">
    <table class="table w-full">
      <thead>
      <tr>
        <th>Play name</th>
        <th>Score</th>
        <th></th>
      </tr>
      </thead>
      <tbody>
      <tr v-for="play in aiStore.stpData.latest?.scoredPlays" :key="play.playName">
<!--      <tr v-for="x in stpStore." :key="name">-->
        <th class="inline-flex items-center gap-1"> <font-awesome-icon v-if="play.playName === aiStore.stpData.latest?.selectedPlay?.playName" icon="circle" class="text-success w-3 h-3" /> {{ play.playName }}</th>
        <td> {{ play.playScore }}</td>
        <td>
          <progress class="progress w-56" :value="play.playScore" max="255"></progress>
        </td>
      </tr>

      </tbody>
    </table>
  </div>
</template>
