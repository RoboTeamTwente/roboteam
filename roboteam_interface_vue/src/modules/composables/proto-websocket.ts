import { ref } from 'vue'
import { proto } from '../../generated/proto'
import MsgToInterface = proto.MsgToInterface

export type WebSocketStatus = 'CLOSED' | 'OPENED' | 'OPENING'
export type OnMessage = (event: MsgToInterface) => void

export const useProtoWebSocket = (onMessage: OnMessage) => {
  let socket: WebSocket | null = null
  const status = ref<WebSocketStatus>('CLOSED')

  const sendProtoMsg = <T extends keyof proto.IMsgFromInterface>(
    kind: T,
    properties: proto.MsgFromInterface[T]
  ) => {
    console.log('Sending', kind, properties)
    const buffer = proto.MsgFromInterface.encode(
      proto.MsgFromInterface.create({
        [kind]: properties
      })
    ).finish()
    socket?.send(buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.length))
  }

  const open = (url: string) => {
    close()

    socket = new WebSocket(url)
    status.value = 'OPENING'

    socket.onopen = () => (status.value = 'OPENED')

    socket.onmessage = async (event) => {
      const messageBuffer = new Uint8Array(await event.data.arrayBuffer())
      const message = proto.MsgToInterface.decode(messageBuffer)
      onMessage(message)
    }

    socket.onclose = () => {
      status.value = 'CLOSED'
      socket = null
    }
  }

  const close = () => {
    // Status code 1000 -> Normal Closure https://developer.mozilla.org/en-US/docs/Web/API/CloseEvent/code
    socket?.close(1000)
  }

  return {
    status,
    open,
    close,
    send: sendProtoMsg
  }
}
