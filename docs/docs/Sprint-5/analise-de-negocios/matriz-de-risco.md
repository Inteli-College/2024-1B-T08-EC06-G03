---
Title: "Matriz de risco"
sidebar_position: 2
---

# Matriz de Riscos

Uma matriz de risco é importante em projetos de tecnologia, ao oferecer uma visão holística e estruturada das potenciais ameaças e oportunidades que podem impactar o planejamento e a execução de uma solução. Ao identificar e classificar os riscos e as oportunidades, desde os prováveis até os mais catastróficos, a equipe de projeto pode antecipar problemas, desenvolver estratégias de mitigação e estabelecer planos de contingência eficazes. Permitindo uma gestão proativa do projeto, garantindo que os recursos sejam alocados adequadamente, os prazos cumpridos e os objetivos alcançados nas expectativas. Além disso, a matriz de risco promove uma comunicação transparente entre os stakeholders, facilitando a tomada de decisões embasadas e a manutenção da confiança no progresso do projeto.

<h6 align="center"> Matriz de Risco </h6>

![Matriz_de_Risco](/img/matriz-de-risco.png)

<h6 align="center"> Fonte: Elaboração feita pelo Grupo Rebólins </h6>

## Riscos

1. Aumentar o tempo necessário para a limpeza.
   - **Explicativa:** Devido a necessidade de adicionar uma etapa ao processo de limpeza, provavelmente o tornará mais lento. Isso se deve, pois o operador deve esperar o robô identificar o tubo para poder limpá-lo. 
   - **Impacto:** Baixo
   - *Justificativa:* Apesar do uso do robô ser uma etapa adicional no processo de limpeza, entende-se que o tempo aumentado não impacta significativamente no processo de limpeza como um todo.
   - **Probabilidade:** 70%
   - *Justificativa:* A probabilidade é alta porque se entende que a movimentação do robô, juntamente com o processo de relimpeza leva mais tempo do que apenas a relimpeza de todos os tubos.
   - **Mitigação**: A equipe de desenvolvimento deve focar em otimizar o tempo de movimentação do robô e no processamento do algoritmo de visão computacional, para que o tempo de limpeza não seja afetado.

2. O robô não é capaz de enviar informações enquanto está na canaleta.
   - **Explicativa:** Como a solução prevê que o robô ande na canaleta da linha de produção, existe o risco do robô não ser capaz de enviar informações no momento que está na canaleta. Isso ocorre devido a dificuldade de acesso ao sinal na linha de produção toda. Com isso, teríamos impacto em uma possível solução web.
   - **Impacto:** Moderado
   - *Justificativa:* Apesar de impossibilitar o envio de informações na linha de produção, não impede que sejam enviadas estatísticas de modo geral, apenas requirindo o uso de outro ambiente com conectividade a internet.
   - **Probabilidade:** 30%
   - *Justificativa:* A probabilidade de 30% é devido a atual não existência de sinal na linha de produção, porém existe o plano para a realização de melhoras na rede de internet nas fábricas.
   - **Mitigação**: A equipe deve buscar tecnologias que driblem esse problema, ou, caso não seja possível, arquiteturas que não necessitem do envio simultâneo de informações.

3. O custo de implementação do protótipo final é mais caro que o parceiro pode arcar.
   - **Explicativa:** Como o custo de implementação é uma variável que não pode ser estimada no momento, é uma ameaça que deve ser considerada na hora de escolher as peças sugeridas.
   - **Impacto:** Alto
   - *Justificativa:* Impediria que o parceiro utilizasse a solução, ou fosse implementada em poucas unidades apenas.
   - **Probabilidade:** 30%
   - *Justificativa:* A probabilidade de 30% é devido ao desconhecimento das peças que serão necessárias para a produção da solução final. 
   - **Mitigação:** A equipe deve buscar por soluções mais baratas, ou buscar por parcerias que ajudem a arcar com os custos.

4. O robô não ser capaz de andar na canaleta.
   - **Explicativa:** Como a solução prevê que o robô ande na canaleta da linha de produção, existe o risco do robô não se adaptar a todas as superfícies presentes nos reboilers.
   - **Impacto:** Alto
   - *Justificativa:* Caso o robô não consiga andar sobre a canaleta, será impossível de realizar a inspeção dos tubos, comprometendo a operação.
   - **Probabilidade:** 50%
   - *Justificativa:* A probabilidade média é porque o modelo inicial de nosso protótipo possui dificuldades para andar em diferentes plataformas.
   - **Mitigação:** A equipe deve focar em otimizar a movimentação do robô ou adicionar elementos especiais às rodas, motores e estabilidade da câmera, para que ele consiga andar em diferentes superfícies.


5. O robô não ser capaz de realizar a leitura correta dos valores visualizados pela câmera.
   - **Explicativa:** É crucial para o funcionamento da solução, que o robô consiga realizar a leitura dos valores obtido pela câmera. Sendo assim, existe a dificuldade de realizar a leitura devido a pouca ou nenhuma luz existente no tubo. Além disso, existe a dificuldade devido a não existência de uma base de dados para a realização da visão computacional.
   - **Impacto:** Muito Alto
   - *Justificativa:* Esse risco tiraria o sentido do projeto, visto que o principal valor do robô é a identificação de pontos que devem ser limpos pelo robô.
   - **Probabilidade:** 30%
   - *Justificativa:* A probabilidade de 30% é devido a não existência da base de dados, o que trará mais dificuldade durante o projeto para a implementação de visão computacional.
   - **Mitigação:** A equipe deve buscar por soluções que permitam a leitura mesmo em ambientes com pouca luz, como uma lanterna periférica, ou buscar por alternativas de análise que não incluam um ambiente com pouca ou nenhuma iluminação.

6. A solução não permite clareza na identificação do tubo que deve ser limpo.
   - **Explicativa:** Uma vez que o robô circulará por baixo, é necessário enviar uma forma de sinal para a pessoa que está limpando o tubo para identificá-lo. 
   - **Impacto:** Muito Alto.
   - *Justificativa:* Caso não seja permitir identificar o tubo, a solução perde seu valor agregado.
   - **Probabilidade:** 50%
   - *Justificativa:* Considerando que a proposta de funcionamento do robô implica que ele funcionará internamente no reboiler, há a dificuldade de visualização por conta da pouca luz dispon
   - **Mitigação:** A equipe deve buscar por soluções que permitam a identificação do tubo, como um sistema de alerta ao operador que está limpando ou modificação do fluxo para o robô inspecionar o tubo antes da limpeza.

## Oportunidades 

1. O reboiler operar de forma mais eficiente após a verificação de limpeza.
   - **Explicativa:** Com a verificação, realizada pelo robô após a limpeza, pretende-se garantir com que não ocorram falhas na limpeza. 
   - **Impacto:** Muito Alto 
   - *Justificativa:* Garantiria que, além do menor número de limpezas necessárias, as paralisações diminuiriam.
   - **Probabilidade:** 50%
   - *Justificativa:* Existe uma chance média do evento ocorrer, visto que não há dados atualmente para fazer a comparação do quão efetiva é a limpeza realizada atualmente.

2. Determinar a periodicidade de limpeza do reboiler.
   - **Explicativa:** Com o processo de verificação dos tubos sujos, é possível verificar quanto tempo em média leva para isso ocorrer. Sendo assim, o processo poderia ocorrer de maneira mais regular, além de aumentar a base de dados da empresa.
   - **Impacto:** Muito Alto
   - *Justificativa:* A descoberta da periodicidade de limpeza do reboiler permitiria que a empresa se planejasse com antecedência.
   - **Probabilidade:** 70%
   - *Justificativa:* Com a implementação de uma verificação e coleta de dados, se torna uma tarefa mais provável de ocorrer.

3. Diminuir custo de manutenção
   - **Explicativa:** A manutenção era realizada apenas quando encontrado problemas de temperaturas altas. Sendo assim, o melhor controle da periodicidade e frequência de manutenção permite evitar deixar o equipamento sem o cuidado necessário. 
   - **Impacto:** Alto
   - *Justificativa:* Uma redução de gastos é sempre importante para empresa, porém não é tão impactante na linha de produção.
   - **Probabilidade:** 70%
   - *Justificativa:* A probabilidade de redução de custos é alta, visto que evitariam falhas e desperdícios de água na manutenção do produto.
    
4. Aumento do uso de tecnologia na empresa, subindo o nível TRL(Technology Readiness Level)
   - **Explicativa:** Com a implementação do robô na fábrica, abrir-se-iam oportunidades para uma maior implementação de tecnologias.
   - **Impacto:** Alto
   - *Justificativa:* O aumento do uso de tecnologia deveria tornar a linha de produção mais eficiente. 
   - **Probabilidade:** 30%
   - *Justificativa:* Há a necessidade de um investimento maior, por parte da empresa, no setor de tecnologia, especialmente em dados, para aumentar o nível e complexidades das soluções desenvolvidas.
  
5. Diminuição no gasto de água, tornando a empresa mais sustentável
   - **Explicativa:** Com a localização do tubo sujo através do robô, deve-se diminuir a quantidade de vezes em que tubos são limpos de forma desnecessária. Sendo assim, trazendo uma economia de água para empresa.
   - **Impacto:** Moderado.
   - *Justificativa:* A redução de água é favorável para empresa tanto para a redução de custos, quanto no ambiental. Porém, como a linha de produção exige um alto gasto de água, o impacto não é tão alto.
   - **Probabilidade:** 70%
   - *Justificativa:* a probabilidade do evento ocorrer é alta, pois o projeto inclui a verificação dos tubos antes da limpeza.
 
6. Realocação de funcionários para tarefas mais adequadas.
   - **Explicativa:** A realocação dos funcionários permitiria que a empresa possa ter um aproveito maior do funcionário, explorando melhor suas habilidades.
   - **Impacto:** Moderado
   - *Justificativa:* O impacto é moderado porque não necessariamente existem outras tarefas para serem realizadas. Porém, há a possibilidade de corte de mão de obra, o que traria uma economia de gastos.
   - **Probabilidade:** 50%
   - *Justificativa:* A probabilidade diz respeito ao tipo de solução desenvolvida, visto que possivelmente o MVP não substituirá o funcionário. Porém, uma possível implementação futura poderia realizar isso.