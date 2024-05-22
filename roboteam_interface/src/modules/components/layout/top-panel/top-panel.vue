<script setup lang="ts">
import { useUIStore } from '../../../stores/ui-store'
import { computed, toRaw } from 'vue'
import { useSTPDataStore } from '../../../stores/data-stores/stp-data-store'
import { useAIDataStore } from '../../../stores/data-stores/ai-data-store'
import { sleep } from '../../../../utils'
import { useAiController } from '../../../composables/ai-controller'
import InfoDropdown from './info-dropdown.vue'

const uiStore = useUIStore()
const stpData = useSTPDataStore()
const aiData = useAIDataStore()
const aiController = useAiController()
const disabled = computed(() => aiController.useReferee)

const haltPlay = () => {
  aiController.currentPlayName = 'Halt'
}

const resetPlay = async () => {
  const tmp = toRaw(aiController.currentPlayName)
  aiController.currentPlayName = 'Halt'
  await sleep(100)
  aiController.currentPlayName = tmp
}

const togglePause = () => {
  aiController.isPaused = !aiController.isPaused
}
</script>
<template>
  <header
    class="top-panel bg-base flex flex-wrap md:flex-nowrap gap-2 lg:gap-4 justify-center border-b-2 border-base-300 items-center p-2"
  >
    <div class="flex w-20">
      <button
        class="btn btn-sm btn-ghost gap-2"
        :class="{ 'btn-active': !uiStore.leftPanel.collapsed }"
        @click="() => uiStore.togglePanel('leftPanel')"
      >
        <font-awesome-icon icon="fa-table-columns" rotation="270" />
      </button>
      <button
        class="btn btn-sm btn-ghost gap-2"
        :class="{ 'btn-active': !uiStore.bottomPanel.collapsed }"
        @click="() => uiStore.togglePanel('bottomPanel')"
      >
        <font-awesome-icon icon="fa-table-columns" rotation="180" />
      </button>
    </div>
    <div class="flex grow" />
    <div class="btn-group">
      <button
        :class="{
          'btn-success': aiController.isPaused,
          'btn-error': !aiController.isPaused
        }"
        class="btn btn-sm gap-2 lg:w-32"
        @click="togglePause"
      >
        <template v-if="!aiController.isPaused">
          <font-awesome-icon icon="fa-square" /> Pause
        </template>
        <template v-else> <font-awesome-icon icon="fa-play" /> Resume </template>
      </button>
    </div>
    <div class="input-group w-auto order-last md:order-none">
      <select
        class="select select-sm sm:w-auto md:w-40 lg:w-auto select-bordered"
        v-model="aiController.currentPlayName"
        :disabled="disabled"
      >
        <option v-for="play in aiData.state!.plays" :value="play" :key="play">
          {{ play }}
        </option>
      </select>
      <select
        class="select select-sm sm:w-auto md:w-24 lg:w-auto select-bordered"
        v-model="aiController.currentRuleset"
        :disabled="disabled"
      >
        <option v-for="ruleset in aiData.state!.ruleSets" :value="ruleset" :key="ruleset">
          {{ ruleset }}
        </option>
      </select>
    </div>
    <div class="btn-group">
      <button
        :disabled='disabled'
        class="btn btn-sm btn-primary gap-2"
        @click="resetPlay"
      >
        <font-awesome-icon icon="fa-rotate-right" /> Reset Play
      </button>
      <button
        :disabled='disabled'
        class="btn btn-sm btn-secondary gap-2"
        @click="haltPlay"
      >
        <font-awesome-icon icon="fa-hand" /> Halt
      </button>
    </div>
    <template v-if="aiController.useReferee">
      <span class="font-mono">
        <template v-if="stpData.latest?.currentPlay?.timeLeft">
          Time left:
          <span class="btn btn-sm gap-2" :style="{ backgroundColor: '#583c7c', color: '#ffffff' }">
            {{stpData.latest.currentPlay.timeLeft}}
          </span>
        </template>
        <template v-if="stpData.latest?.currentPlay?.commandFromRef">
          Current ref command:
          <span class="btn btn-sm gap-2" :style="{ backgroundColor: '#583c7c', color: '#ffffff' }">
            {{stpData.latest.currentPlay.commandFromRef}}
          </span>
        </template>
        <template v-if="stpData.latest?.currentPlay?.followUpCommandFromRef">
          Next ref command:
          <span class="btn btn-sm gap-2" :style="{ backgroundColor: '#583c7c', color: '#ffffff' }">
            {{stpData.latest.currentPlay.followUpCommandFromRef}}
          </span>
        </template>
      </span>
    </template>
    <div class="flex grow" />
    <div class="flex w-20 justify-end">
      <info-dropdown
        :current-tick="stpData.latest?.currentTick ?? -1"
        :tick-duration="stpData.latest?.tickDuration ?? -1"
        :avg-tick-duration="stpData.latest?.averageTickDuration ?? -1"
        @disconnect-from-ai="aiController.close"
      />
    </div>
  </header>
</template>

<style scoped></style>
