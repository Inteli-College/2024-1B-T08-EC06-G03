# Análise Financeira

## Introdução

A análise financeira é uma ferramenta fundamental para avaliar a situação econômica e financeira de uma empresa, fornecendo informações cruciais para a tomada de decisões estratégicas. Esta análise permite identificar pontos fortes e fracos, oportunidades e ameaças, facilitando a elaboração de planos de ação que maximizem a eficiência e a lucratividade.

No contexto do projeto "Desencana!", a análise financeira é vital para estimar os custos associados à prototipação e desenvolver a proposta mais viável para nossos parceiros. Este processo abrange desde a prova de conceito até a solução final idealizada, garantindo que todas as etapas sejam economicamente sustentáveis e alinhadas com os objetivos estratégicos da empresa. A análise detalhada dos custos e benefícios permitirá otimizar os recursos, minimizar riscos e assegurar o sucesso do projeto.

## Prova de Conceito

### Estimativa de custos

#### Tabela de preços líquidos (custos iniciais de capital):

| Produto | Preço (R$) |
|---------|-------|
| TurtleBot3 Burger RPi4 2GB      | 3.381,81    |
| Webcam Logitech c270i 720P           | 120,00     |
| Aparelho para usar o software          | 1.869,00     |

Esta tabela contempla os custos de capital necessários para a implementação da prova de conceito, incluindo o robô (convertido com uma taxa de câmbio de 1 USD = 5,12 BRL), a webcam e o aparelho para usar o software, no caso um notebook com uma configuração mínima.

#### Cálculo do preço bruto do robô:

Considerando o preço líquido do _TurtleBot3 Burger RPi4 2GB_ como R$ 3.381,81 (com uma taxa de câmbio de 1 USD = 5,12 BRL) de acordo com o site [Robotis](https://robotis.us), e as taxas de frete (shipping) e imposto de importação de 60% do total pago, de acordo com o [Afinz blog](https://afinz.com.br/blog/mercado/taxa-de-importacao-2024/), é possível fazer a seguinte relação:

```
Preço bruto = Preço líquido + Frete + Imposto de importação
Preço bruto = 3.381,81 + 2.137,78 + (3.381,81 * 0,6)
Preço bruto = 3.381,81 + 2.137,78 + 2.029,08
Preço bruto = R$ 7.548,67
```

#### Custos recorrentes:

Considerando a necessidade de uma hospedagem para o software pela AWS (Amazon Web Services), foi escolhido o plano de hospedagem da AWS (Amazon Web Services) utilizando o serviço Amazon Elastic Cloud Computing (EC2), que tem um custo mensal de aproximadamente R$ 365,20, caso sejam utilizadas instâncias mais básicas, como a _c5.large_. O custo anual é calculado multiplicando o custo mensal por 12 meses:

```
Custo mensal = R$ 365,20
365,20 * 12 = R$ 4.382,47
```
Total de custos recorrentes em um ano: R$ 4.382,47

#### Valor final da implementação (com fretes e custos adicionais inclusos):

| Produto | Preços finais (R$) |
|---------|-------|
| TurtleBot3 Burger RPi4 2GB      | 7.548,67    |
| Webcam Logitech c270i 720P           | 133,00     |
| Aparelho para usar o software          | 1.869,00     |
| Hospedagem do software          | 4.382,47/ano     |
| **Total**          | 13.933,14     |

**Nota:** _Os custos de capital foram calculados com base nos preços dos produtos e serviços no mercado atual, e os custos recorrentes foram calculados com base nos valores de hospedagem da AWS (Amazon Web Services)._

**Referências:**
- [Imposto de importação](https://afinz.com.br/blog/mercado/taxa-de-importacao-2024/)
- [Turtlebot3 Burger RPi 3 1 GB](https://www.robotis.us/turtlebot-3-burger-rpi4-2gb-us/)
- [Webcam Logitech c270i 720P](https://shopee.com.br/product/262478502/22330419188?gsht=MaNhXjMRjbKbgyH1&srsltid=AfmBOopaSwARWyrtlfVf-7e3quru0LbAZvq_vpcg_x0qEs1xuaAutDcxfTU)
- [Notebook Ultra UB445](https://www.kabum.com.br/produto/393587/notebook-ultra-com-windows-11-home-processador-intel-core-i3-8gb-240gb-ssd-tela-14-1-pol-hd-tecla-netflix-prata-ub445) (aparelho para usar o software)

### Investimento inicial

Para o primeiro ano da utilização do projeto "Desencana!", o investimento inicial mínimo esperado é de R$ 14.000,00, considerando os custos de capital inicial e os custos recorrentes.

### Benefícios esperados

Os benefícios financeiros do projeto "Desencana!" incluem mas não se limitam a:

- Otimização de processos de limpeza e manuntenção de ambientes, gerando redução de custos operacionais;
- Aumento da eficiência e produtividade dos funcionários;
- Melhoria da qualidade de vida dos funcionários, que podem se dedicar a tarefas mais estratégicas e menos operacionais;
- Melhoria na qualidade dos serviços prestados, gerando maior satisfação dos clientes.

### Retorno sobre o investimento (ROI)

O retorno sobre o investimento (ROI) é uma métrica financeira que mede a eficiência de um investimento. O ROI é calculado pela fórmula:

```markdown
ROI = (Ganho obtido - Custo do investimento) / Custo do investimento
```

Para o projeto "Desencana!", o ROI é calculado considerando o investimento inicial de R$ 14.000,00 e os benefícios esperados ao longo de um ano. Juntando estas informções, obtém-se:

```markdown
ROI = (Benefícios esperados - Custo do investimento) / Custo do investimento
ROI = (Benefícios esperados - R$ 14.000,00) / R$ 14.000,00
```

Como os benefícios esperados são difíceis de quantificar com precisão, o ROI do projeto "Desencana!" será calculado com base em uma estimativa conservadora de R$ 20.000,00 no primeiro ano. Com isso, é possível obter o seguinte resultado:

```markdown
ROI = (R$ 20.000,00 - R$ 14.000,00) / R$ 14.000,00
ROI = R$ 6.000,00 / R$ 14.000,00
ROI = 0,4286 ou 42,86%
```

### Conclusão

A análise financeira da prova de conceito do projeto "Desencana!" demonstra que o investimento inicial de R$ 14.000,00 tem o potencial de gerar benefícios anuais de R$ 20.000,00, resultando em um ROI de 42,86%. Este ROI significativo indica que o projeto é financeiramente viável e promete um retorno positivo para a Atvos. A implementação do "Desencana!" não apenas otimiza a eficiência operacional e reduz os tempos de inatividade não planejados, mas também reforça o compromisso da Atvos com a inovação e a excelência. Portanto, o projeto representa uma oportunidade estratégica para a empresa, com benefícios econômicos claros e sustentáveis, posicionando a Atvos na vanguarda da indústria de processamento de cana-de-açúcar.

### Alternativas para diminuição da proposta

Para diminuir o custo da proposta, é possível considerar as seguintes estratégias:

#### 1. **Otimização do Hardware**
- **Substituição por Componentes Mais Econômicos:** Avaliar a possibilidade de utilizar componentes de hardware alternativos que oferecem funcionalidades similares a um custo menor. Por exemplo, investigar se há modelos de robôs e câmeras mais acessíveis que ainda atendam aos requisitos do projeto.
- **Compra em Volume:** Negociar descontos por compra em volume com fornecedores de hardware. Compras em grandes quantidades geralmente permitem obter preços mais baixos.

#### 2. **Reavaliação dos Serviços de Hospedagem**
- **Escolha de Planos Mais Econômicos:** Revisar os planos de hospedagem do AWS e optar por opções mais econômicas que ainda atendam às necessidades da implementação. Por exemplo, utilizar instâncias de EC2 menores ou otimizadas para o custo-benefício, e explorar planos de armazenamento que oferecem custos reduzidos.
- **Utilização de Serviços Gerenciados:** Avaliar se o uso de serviços gerenciados da AWS pode reduzir os custos operacionais e de manutenção em longo prazo.

#### 3. **Revisão dos Custos Recorrentes**
- **Análise de Utilização:** Monitorar e ajustar a utilização dos recursos em tempo real para evitar custos desnecessários com serviços em nuvem. Por exemplo, desligar instâncias quando não estiverem em uso.
- **Contratação de Serviços em Longo Prazo:** Aproveitar descontos oferecidos pela AWS para compromissos de longo prazo (como as instâncias reservadas), que podem oferecer economias significativas em comparação com a cobrança por demanda.

#### 4. **Parcerias e Subsídios**
- **Parcerias Estratégicas:** Buscar parcerias com outras empresas ou instituições que possam fornecer recursos, conhecimento ou infraestrutura a um custo reduzido ou até mesmo gratuitamente.
- **Programas de Subsídios e Incentivos:** Investigar programas governamentais ou de instituições privadas que ofereçam subsídios ou incentivos para projetos inovadores em áreas como robótica e visão computacional.

#### Exemplo de Aplicação Prática

**Cenário Atual:**
- TurtleBot3 Burger RPi4 2GB: R$ 7.548,67
- Webcam: R$ 133,00
- Aparelho para usar o software: R$ 1.869,00
- Hospedagem do software: R$ 365,20/mês (R$ 4.382,47/ano)
- **Total:** R$ 13.933,14

**Cenário Otimizado:**
- Substituição do TurtleBot3 por um robô semelhante alternativo de custo R$ 5.000,00
- Substituição da webcam por uma versão de custo R$ 80,00
- Redução dos custos do aparelho para usar o software para R$ 1.500,00
- Escolha de um plano de hospedagem mais econômico que reduza o custo anual para R$ 1.800,00

**Novo Total:**
- Robô: R$ 5.000,00
- Webcam: R$ 80,00
- Aparelho: R$ 1.500,00
- Hospedagem anual: R$ 3.500,00
- **Novo Total:** R$ 10.080,00

Com essas alterações, o valor da proposta é reduzido de R$ 14.000,00 para R$ 10.080,00, mantendo a funcionalidade e eficiência do projeto "Desencana!".

## Solução Final