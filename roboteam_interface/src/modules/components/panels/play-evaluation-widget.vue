<script setup lang="ts">
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { useSTPDataStore } from '../../stores/data-stores/stp-data-store'

const stpData = useSTPDataStore()
</script>

<template>
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
        <tr v-for="play in stpData.latest?.scoredPlays" :key="play.playName!">
          <th class="inline-flex items-center gap-1">
            <font-awesome-icon
              v-if="play.playName === stpData.latest?.currentPlay?.playName"
              icon="circle"
              class="text-success w-3 h-3"
            />
            {{ play.playName }}
          </th>
          <td>{{ play.playScore }}</td>
          <td>
            <progress class="progress w-56" :value="play.playScore!" max="255"></progress>
          </td>
        </tr>
      </tbody>
    </table>
  </div>
</template>
