<script setup lang="ts">
import { ref } from 'vue'
import { WebSocketStatus } from '../composables/proto-websocket'

const props = defineProps<{
    status: WebSocketStatus
  }>(),
  emit = defineEmits<{
    (e: 'connect', url: string): void
  }>(),
  url = ref('localhost:12676')
</script>

<template>
  <div class="modal" :class="{ 'modal-open': props.status !== 'OPENED' }">
    <div class="modal-box flex flex-col gap-4">
      <div class="m-auto">
        <img src="/favicon.svg" class="w-32" />
      </div>

      <span class="flex items-center"
        >Status:
        <span
          class="inline-block rounded-full w-2 h-2 bg-red-600 ml-2 mr-1"
          :class="{
            '!bg-green-600': props.status == 'OPENED',
            '!bg-yellow-600': props.status == 'OPENING'
          }"
        />
        {{ props.status }}
      </span>
    <form @submit.prevent="() => emit('connect', 'ws://' + url)">
      <div class="form-control w-full max-w">
        <label class="label">
          <span class="label-text">AI InterfaceGateway url</span>
        </label>
        <label class="input-group">
          <span>ws://</span>
          <input class="input input-bordered w-full" v-model="url" />
        </label>
      </div>
      <div class="form-control w-full max-w">
        <button class="btn btn-primary w-full" type="submit">Connect</button>
      </div>
    </form>
    </div>
  </div>
</template>
