# Desencana!
Repositório do grupo Rebólins

## Descrição do Projeto "Desencana!"
O grupo Rebólins, em parceria com a Atvos, comprometida com a melhoria contínua de suas operações na indústria de processamento de cana-de-açúcar, está implementando uma solução inovadora de monitoramento e manutenção de reboilers através do "Projeto Desencana!". Este projeto visa integrar as tecnologias de robótica e visão computacional clássica para inspecionar e verificar a eficácia da limpeza dos tubos de reboilers, essenciais para a etapa de processamento da cana. Utilizando um sistema de sensores auxiliares, o robô especializado realiza uma avaliação precisa do estado de limpeza dos tubos, coletando dados críticos, que são automaticamente enviados para uma base central. Essa iniciativa não só otimiza a eficiência operacional, reduzindo tempos de inatividade devido a manutenções não planejadas, mas também garante a manutenção da qualidade e segurança no processo de produção. Através do Projeto Desencana!, a Atvos reforça seu compromisso com a inovação e a excelência operacional, estabelecendo novos padrões de manutenção preventiva na indústria.
## Objetivos do Projeto
- Garantir que todos os reboilers estão completamente limpos;
- Disponibilizar de forma sistemática dados relacionados a limpeza dos tubos;

## 👨‍🎓 Integrantes: 
- <a href="https://www.linkedin.com/in/eduardosbarreto/">Eduardo Santos Barreto</a>
- <a href="https://www.linkedin.com/in/fernando-vasconcellos-/">Fernando Antonio Sampaio Cabral de Vasconcelos</a>
- <a href="https://www.linkedin.com/in/eduardo-franca-porto/">Gabrielle Dias Cartaxo</a>
- <a href="https://www.linkedin.com/in/guilherme-ferreira-linhares-8638411a1/">Guilherme Ferreira Linhares</a>
- <a href="https://www.linkedin.com/in/naruto/">Ivan Gonçalves Ferreira</a>
- <a href="https://www.linkedin.com/in/luiza-rubim/">Luiza Souza Rubim</a>
- <a href="https://www.linkedin.com/in/olincosta/">Ólin Medeiros Costa</a>

## 👩‍🏫 Professores:
### Orientador
- <a href="https://www.linkedin.com/in/rodrigo-mangoni-nicola-537027158/">Rodrigo Nicola</a>
### Instrutores
- <a href="https://www.linkedin.com/in/gui-cestari/">Guilherme Cestari</a>
- <a href="https://www.inteli.edu.br/">Geraldo Vasconcelos</a> 
- <a href="https://www.linkedin.com/in/lisane-valdo/">Lisane Valdo</a> 
- <a href="https://www.linkedin.com/in/monica-anastassiu-d-sc-2568522/">Mônica Anastassiu</a>
- <a href="https://www.linkedin.com/in/murilo-zanini-de-carvalho-0980415b/">Murilo Zanini de Carvalho</a>

## Estrutura de Pastas
```
└── 📂2024-1B-T08-EC06-G03
    └── 📂.github
        └── 📂workflows
            └── 📜static.yml
    └── 📂docs
        └── 📂.docusaurus
        └── 📜.gitignore
        └── 📜babel.config.js
        └── 📂docs
            └── index.md
            └── 📂Sprint 1
                └── 📂Arquitetura de Solucao
                └── 📂Design
                └── 📂Negocios
            └── 📂Sprint 2
            └── 📂Sprint 3
            └── 📂Sprint 4
            └── 📂Sprint 5
        └── 📜docusaurus.config.js
        └── 📜package-lock.json
        └── 📜package.json
        └── 📜README.md
        └── 📂src
            └── 📂components
                └── 📂HomepageFeatures
            └── 📂css
        └── 📂static
            └── 📂img
        └── README.md
    ├── 📂src
        └── 📂frontend
           └── 📂public
            └── 📂css
                └── 📜style.css
            └── 📂img
                └── 📜noSignal.jpg
            └── 📂js
                └── 📜script.js
           ┗ 📜index.html
        └──📂workspace
            └── 📂src
               └── 📂robot_navigation
                   └── 📂resource
                      └── 📜robot_navigation
                   └── 📂robot_navigation
                      └── 📜__init__.py
                      └── 📜bot.py
                   └── 📂test
                      └── 📜test_copyright.py
                      └── 📜test_flake8.py
                      └──📜test_pep257.py
                      └── 📜package.xml
                    └── 📜setup.cfg
                    └── 📜setup.py
        └── 📜run.sh
    └── 📜README.md
    └── 📜.gitignore
```

## Execução do Projeto

É necessário clonar o projeto para iniciar a execução do projeto.

```sh
git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G03.git
```
### 1. Conexão com o Robô via SSH
Para iniciar a conexão com o robô, é necessário utilizar o protocolo SSH. Há um tutorial de como deve ser feito o [setup do robô](https://inteli-college.github.io/2024-1B-T08-EC06-G03/Sprint%202/Metodologia) Execute os seguintes passos no seu terminal:

1. **Inicie o Robô**: Certifique-se de que o robô esteja ligado e pronto para conexão.
2. **Conectar via SSH**:
   - Abra um terminal no seu computador.
   - Digite o comando SSH para estabelecer uma conexão segura. Confirme o comando exato e o IP na documentação de metodologia. Um exemplo de comando é:
     ```bash
     ssh -p 1238 bobolins@10.128.0.16
     ```

### 2. Inicialização do Robô
Após estabelecer a conexão SSH, o próximo passo é inicializar os componentes necessários do robô.

1. **Executar o Comando de Bring Up**:
   - No terminal SSH, execute o comando que inicializa os processos necessários no robô. Este comando pode ser encontrado na documentação técnica ou em tutoriais relevantes. Um exemplo comum para robôs baseados em ROS pode ser algo como:
     ```bash
     ros2 launch turtlebot3_bringup robot.launch.py
     ```
   - Este comando inicializa um listener no robô e ativa vários serviços e processos internos.

### 3. Execução do Software de Controle
Com o robô devidamente inicializado, a próxima etapa é executar o software de controle que interage com o robô.

1. **Preparar o Ambiente de Software**:
   - No seu computador (não no terminal SSH), abra um novo terminal.
   - Navegue até o diretório do projeto e entre na pasta do código fonte 
     ```bash
     cd src/workspace
     ```
   - Execute o script que inicia o servidor ou o software de controle:
     ```bash
     ./run.sh
     ```

2. **Abrir Interface de Controle**:
   - Localize o arquivo HTML responsável pela interface de controle. O caminho exato deve ser verificado na documentação do projeto.
   - Abra o arquivo HTML diretamente com um navegador para acessar a interface de controle do robô. É indicado abrir através da extensão `Live Server`, porém pode ser aberto como um arquivo normal, apenas executando ele.

### 4. Controle do Robô
- Utilize as teclas especificadas na interface HTML para controlar o robô.

**Nota:** É importante confirmar todos os comandos e caminhos exatos com a documentação técnica disponível para garantir que as instruções estejam corretas e atualizadas.

## Documentação

Para acessar a nossa [documentação](https://inteli-college.github.io/2024-1B-T08-EC06-G03/), clique [aqui](https://inteli-college.github.io/2024-1B-T08-EC06-G03/)!

## 📋 Licença/License
<p xmlns:cc="http://creativecommons.org/ns#" xmlns:dct="http://purl.org/dc/terms/"><span property="dct:title">Desencana!</span> - por <span property="cc:attributionName"> <a href="https://www.linkedin.com/in/eduardosbarreto/">Eduardo Barreto</a>, <a href="https://www.linkedin.com/in/fernando-vasconcellos-/">Fernando Vasconcelos</a>, <a href="https://www.linkedin.com/in/eduardo-franca-porto/">Gabrielle Cartaxo</a>, <a href="https://www.linkedin.com/in/guilherme-ferreira-linhares-8638411a1/">Guilherme Linhares</a>, <a href="https://www.linkedin.com/in/naruto/">Ivan Ferreira</a>, <a href="hhttps://www.linkedin.com/in/luiza-rubim/">Luiza Rubim</a> e <a href="https://www.linkedin.com/in/olincosta/">Olin Costa</a></span> is licensed under <a href="http://creativecommons.org/licenses/by/4.0/?ref=chooser-v1" target="_blank" rel="license noopener noreferrer" style="display:inline-block;">CC BY 4.0<img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1"><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1"></a></p>
