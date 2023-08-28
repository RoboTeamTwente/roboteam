<script setup lang="ts">
import { useRefereeDataStore } from '../../stores/data-stores/referee-store'
import { sslRefCommandToIconName, sslRefCommandToString, stageToString } from '../../../utils'
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'

const refereeData = useRefereeDataStore()

const matchTime = (timestamp: number) => {
  const date = new Date(timestamp / 1000)
  return `${date.getHours()}:${date.getMinutes()}:${date.getSeconds()}`
}
</script>

<template>
  <table class="table w-full">
    <thead>
      <tr>
        <th class="border-b">Timestamp</th>
        <th class="border-b">Command</th>
        <th class="border-b">Stage</th>
      </tr>
    </thead>
    <tbody>
      <tr v-for="(cmd, index) in refereeData.data.slice().reverse()" :key="index">
        <td class="border-b">
          {{ matchTime(cmd.commandTimestamp) }}
        </td>
        <td class="border-b">
          <div
            class="bg-base-200 rounded-xl border p-1 px-2 font-mono font-bold text-sm w-min flex items-center gap-2 whitespace-nowrap"
          >
            <font-awesome-icon :icon="sslRefCommandToIconName(cmd.command)" class="w-3 h-3" />
            {{ sslRefCommandToString(cmd.command) }}
          </div>
        </td>
        <td class="border-b">
          <div
            class="bg-base-200 rounded-xl border p-1 px-2 font-mono font-bold text-sm w-min whitespace-nowrap"
          >
            {{ stageToString(cmd.stage) }}
          </div>
        </td>
      </tr>
    </tbody>
  </table>
</template>
