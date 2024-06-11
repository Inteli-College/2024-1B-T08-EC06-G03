---
title: Guia de Execuções
sidebar_position: 4
---

# Guia de instrução 

## Conexão com o robô

### 1. Conexão com o Robô via SSH
Para iniciar a conexão com o robô, é necessário utilizar o protocolo SSH. Há um tutorial de como deve ser feito o [setup do robô](https://inteli-college.github.io/2024-1B-T08-EC06-G03/Sprint%202/Metodologia) Execute os seguintes passos no seu terminal:

1. **Inicie o Robô**: Certifique-se de que o robô esteja ligado e pronto para conexão.
2. **Conectar via SSH**:
   - Abra um terminal no seu computador.
   - Digite o comando SSH para estabelecer uma conexão segura. Confirme o comando exato e o IP na documentação de metodologia. Um exemplo de comando é:
     ```bash
     ssh rebolins@rebolins.local
     ```

### 2. Clonar o projeto no robô 

```sh
git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G03.git
```
Além disso, é importante ter instalado o [ROS2](https://docs.ros.org/en/humble/Installation.html).

### 3. Inicialização do Robô
Após estabelecer a conexão SSH, o próximo passo é inicializar os componentes necessários do robô. Execute isso no mesmo terminal do 

1. **Entrar na pasta de código**:
    - É necessário entrar na pasta que os pacotes estão alocados.
    ```bash
    cd src/bolin
    ```

2. **Executar o Build.sh**:
    - É necessário rodar o arquivo de build para construir os pacotes ros.
    ```bash
    source build.sh
    ```

3. **Executar o Comando de Bring Up**:
   - No terminal SSH, execute o comando que inicializa os processos necessários no robô. Este comando pode ser encontrado na documentação técnica ou em tutoriais relevantes. Um exemplo comum para robôs baseados em ROS pode ser algo como:
     ```bash
     ros2 launch bolin_bringup launch.py
     ```
   - Este comando inicializa um listener no robô e ativa vários serviços e processos internos.

### 3. Execução do Software de Controle
Com o robô devidamente inicializado, a próxima etapa é executar o software de controle que interage com o robô.

1. **Inicialização do Back-End**:
    - Instale o [Node.js](https://nodejs.org/en/download/)
    - No seu computador (não no terminal SSH), abra um novo terminal.
    - Acesse a pasta do Back-End

    ```bash
    cd src/backend/
    ```

    - Instale as dependências

    ```bash
    npm install
    ```
    :::warning Alerta
    Para continuar, visualize as intruções existentes para a [configuração do database](/Sprint%204/banco_de_dados.md), pois é necessário gerar o `schema.prisma` para que o back-end execute.    
    :::

    - Execute o Back-End

    ```bash
    npm start
    ```
    
2. **Inicialização do Front-End**:
    - Instale o [Node.js](https://nodejs.org/en/download/)
    - No seu computador (não no terminal SSH), abra um novo terminal.
    - Acesse a pasta do Front-End

    ```bash
    cd src/frontend/
    ```

    - Instale as dependências

    ```bash
    npm install
    ```

    - Execute o Front-End

    ```bash
    npm run dev
    ```

    - Acesse [http://localhost:5173](http://localhost:5173) no seu navegador.

### 4. Controle do Robô
- Utilize o joystick para controlar o robô.


**Nota:** É importante confirmar todos os comandos e caminhos exatos com a documentação técnica disponível para garantir que as instruções estejam corretas e atualizadas.
