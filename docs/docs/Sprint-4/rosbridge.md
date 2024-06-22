---
title: Migração para WebSockets com ROSBridge
sidebar-position: 5
---

# Migração para websockets com ROSBridge

## Contexto

Nas últimas sprints, enfrentamos problemas significativos de latência e instabilidade na visualização da câmera do nosso robô. Anteriormente, o backend do projeto instanciava nodes ROS que atuavam como subscribers da câmera e da teleoperação do robô, e então se comunicavam via WebSockets com o frontend. Essa abordagem resultava em alta latência e instabilidade na visualização da câmera, além de exigir que o ROS fosse instalado na máquina que executa o backend.

## A Solução: ROSBridge

Para resolver esses problemas, decidimos migrar para uma arquitetura que utiliza ROSBridge para a comunicação via WebSockets. A implementação dessa solução resultou em uma visualização da câmera mais estável e com menor latência.

Os principais benefícios observados foram:

1. **Redução de Latência:** A comunicação direta via ROSBridge diminuiu significativamente a latência na transmissão de imagens.
2. **Estabilidade Melhorada:** A visualização da câmera se tornou mais estável, sem interrupções frequentes.
3. **Independência do Backend:** Eliminar a necessidade de instalar ROS na máquina do backend simplificou a infraestrutura do sistema.

## Como Funciona

### Arquitetura

1. **Envio de Imagens pela Câmera do Robô:** A câmera do robô captura imagens e envia diretamente para um tópico do ROS.
2. **Middleware ROSBridge:** O ROSBridge atua como um intermediário, recebendo as imagens do tópico e disponibilizando-as para aplicações web através de WebSockets.
3. **Aplicação Web:** A aplicação web se conecta ao ROSBridge, recebe as imagens e as exibe em tempo real.

## Demonstração

Para testar e ilustrar a eficiência dessa nova arquitetura, fizemos um teste indo do laboratório até a biblioteca. Abaixo está o link para o vídeo:

<iframe width="560" height="315" src="https://www.youtube.com/embed/KSn_qqKmkII?si=MUpYGWARh5Fxo90e" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Apesar de apresentar uma latência grande em alguns momentos, realizar essa movimentação com a arquitetura antiga era impensável.

---

Seguindo estas etapas e considerando os benefícios mencionados, a migração para WebSockets com ROSBridge não só melhorou a performance do nosso sistema como também simplificou sua manutenção e escalabilidade.
