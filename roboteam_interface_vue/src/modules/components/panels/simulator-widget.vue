<script setup lang="ts">
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { computed, toRaw } from 'vue'
import { useAiController } from '../../composables/ai-controller'

const aiController = useAiController()
const disabled = computed(() => aiController.useReferee)

const sendSimulatorCommand = aiController.sendSimulatorCommand

</script>

<template>
  <div class="mt-4 flex flex-col gap-4 max-w-md">
    <button
    :disabled='disabled'
    class="btn btn-sm btn-secondary gap-2"
        @click="sendSimulatorCommand('robotsToSide')"
    >
        <font-awesome-icon icon="fa-arrows" />
        Move robots to side
    </button>
    
    <button
        :disabled='disabled'
        class="btn btn-sm btn-secondary gap-2"
        @click="sendSimulatorCommand('ballToCenter')"
    >
        <font-awesome-icon icon="fa-crosshairs" />
        Move ball to center
    </button>
    

  </div>
</template>