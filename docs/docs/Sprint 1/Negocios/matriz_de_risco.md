# Matriz de Riscos

Uma matriz de risco é importante em projetos de tecnologia, pois oferece uma visão holística e estruturada das potenciais ameaças e oportunidades que podem impactar o planejamento e a execução de uma solução. Ao identificar e classificar os riscos e as oportunidades, desde os mais prováveis até os mais catastróficos, a equipe de projeto pode antecipar problemas, desenvolver estratégias de mitigação e estabelecer planos de contingência eficazes. Permitindo uma gestão proativa do projeto, garantindo que os recursos sejam alocados adequadamente, os prazos cumpridos e os objetivos alcançados dentro das expectativas. Além disso, a matriz de risco promove uma comunicação transparente entre os stakeholders, facilitando a tomada de decisões embasadas e a manutenção da confiança no progresso do projeto.

![Matriz_de_Risco](/img/matriz-de-risco.png)
<h6 align="center"> Fonte: Elaboração feita pelo Grupo Rebólins </h6>

## Riscos

1. Aumentar o tempo necessário para a limpeza
   - **Explicativa:** Devido a necessidade de adicionar uma etapa ao processo de limpeza, provavelmente o tornará mais lento. Isso se deve pois o operador deve esperar o robô identificar o tubo para poder limpá-lo. 
   - **Impacto:** Baixo
   - *Justificativa:* Apesar do uso do robô ser uma etapa adicional no processo de limpeza, entende-se que o tempo aumentado não impacta significativamente no processo de limpeza como um todo.
   - **Probabilidade:** 70%
   - *Justificativa:* A probabilidade é alta porque entende-se que a movimentação do robô, juntamente com o processo de relimpeza leva mais tempo do que apenas a relimpeza de todos os tubos.

2. O robô não é capaz de enviar informações enquanto está na canaleta
   - **Explicativa:** Como a solução prevê que o robô ande na canaleta da linha de produção, existe o risco do robô não ser capaz de enviar informações no momento que está na canaleta. Isso ocorre devido a dificuldade de acesso ao sinal na linha de produção como um todo. Com isso, teríamos impacto em uma possível solução web.
   - **Impacto:** Moderado
   - *Justificativa:* Apesar de impossibilitar o envio de informações na linha de produção, não impede que sejam enviadas estatísticas de modo geral, apenas requerindo o uso de um outro ambiente com conectividade a internet.
   - **Probabilidade:** 30%
   - *Justificativa:* A probabilidade de 30% é devido a atual não existência de sinal na linha de produção, porém existe o plano para a realização de melhoras na rede de internet nas fábricas.

3. O custo de implementação do protótipo final é mais caro que o parceiro é capaz de arcar.
   - **Explicativa:** Como o custo de implementação é uma variável que não pode ser estimada no momento, é uma ameaça que deve ser considerada na hora de escolher as peças sugeridas.
   - **Impacto:** Alto
   - *Justificativa:* Impediria que o parceiro utilizasse a solução, ou fosse implementada em poucas unidades apenas.
   - **Probabilidade:** 30%
   - *Justificativa:* A probabilidade de 30% é devido ao desconhecimento das peças que serão necessárias para a produção da solução final. 

4. O robô não ser capaz de andar na canaleta
   - **Explicativa:** Como a solução prevê que o robô ande na canaleta da linha de produção, existe o risco do robô não se adaptar a todas as superfícies presentes nos rebóilers.
   - **Impacto:** Alto
   - *Justificativa:* Caso o robô não consiga andar sobre a canaleta, será impossível de realizar a inspeção dos tubos, comprometendo a operação.
   - **Probabilidade:** 50%
   - *Justificativa:* A probabilidade média é devido ao fato de que o modelo inicial de nosso protótipo possui dificuldades para andar em diferentes plataformas.

5. Implementação da movimentação autônoma do robô não ser concluída
   - **Explicativa:** A solução prevê a implementação da movimentação do robô de forma autônoma. Porém a implementação de um caminho para circular de forma autônoma é complexo e custoso.
   - **Impacto:**  Alto
   - *Justificativa:* Caso o robô não seja capaz de se movimentar de forma autônoma, isso exige que ele seja operado de forma remota, assim ainda trazendo dificuldades para o desenvolvimento da solução. Além disso, traz riscos de falta de comunicação entre o operador e o robô.
   - **Probabilidade:** 70%
   - *Justificativa:* A probabilidade é julgada alta, por não saber exatamente o caminho que o robô deve percorrer, assim como pela complexidade de implementação.


6. O robô não ser capaz de realizar a leitura correta dos valores visualizados pela câmera.
   - **Explicativa:** É crucial para o funcionamento da solução, que o robô seja capaz de realizar a leitura dos valores obtido pela câmera. Sendo assim, existe a dificuldade de realizar a leitura devido a pouca ou nenhuma luz existente no tubo. Além disso, existe a dificuldade devido a não existência de uma base de dados para a realização da visão computacional.
   - **Impacto:** Muito Alto
   - *Justificativa:* Esse risco tiraria o sentido do projeto, visto que o principal valor do robô é a identificação de pontos que devem ser limpos pelo robô.
   - **Probabilidade:** 30%
   - *Justificativa:* A probabilidade de 30% é devido a não existência da base de dados, o que trará mais dificuldade durante o projeto para a implementação de visão computacional.

7. A solução não permite clareza na identificação do tubo que deve ser limpo.
   - **Explicativa:** Uma vez que o robô circulará por baixo, é necessário que envie uma forma de sinal para a persona que está limpando o tubo para indetinficá-lo. 
   - **Impacto:** Muito Alto
   - *Justificativa:* Caso não seja permitir indentificar o tubo, a solução perde seu valor agregado.
   - **Probabilidade:** 50%
   - *Justificativa:* A probabilidade disso depende da solução de arquitetura desenvolvida pelo grupo. Como ainda não há proposta para a resolução desse problema, a probabilidade se torna de 50%.

## Oportunidades 

1. O reboiler operar de forma mais eficiente após a verificação de limpeza.
   - **Explicativa:** Com a verificação, realizada pelo robô após a limpeza, pretende-se garantir com que não ocorram falhas na limpeza. 
   - **Impacto:** Muito Alto 
   - *Justificativa:* Garantiria que, além do menor número de limpezas necessárias, as paralisações diminuiriam.
   - **Probabilidade:** 50%
   - *Justificativa:* Existe uma chance média do evento ocorrer, visto que não há dados atualmente para fazer a comparação do quão efetiva é a limpeza realizada atualmente.

2. Determinar a periodicidade de limpeza do reboiler.
   - **Explicativa:** Com o processo de verificação dos tubos sujos, é possível verificar quanto tempo em média leva para isso ocorrer. Sendo assim, o processo poderia ocorrer de maneira mais regular, além de aumentar a base de dados da empresa
   - **Impacto:** Muito Alto
   - *Justificativa:* A descoberta da periodicidade de limpeza do reboiler permitiria que a empresa se planejasse com antecedência.
   - **Probabilidade:** 70%
   - *Justificativa:* Com a implementação de uma verificação e coleta de dados, se torna uma tarefa mais provável de ocorrer.

3. Diminuir custo de manutenção
   - **Explicativa:** A manutenção era realizada apenas quando encontrado problemas de temperatura altas. Sendo assim, o melhor controle da periodicidade e frequência de manutenção permite evitar deixar o equipamento sem o cuidado necessário. 
   - **Impacto:** Alto
   - *Justificativa:* Uma redução de gastos é sempre importante para empresa, porém não é tão impactante na linha de produção.
   - **Probabilidade:** 70%
   - *Justificativa:* A probabilidade de redução de custos é alta, visto que evitariam falhas e desperdícios de água na manutenção do produto.
  
4. Aumento do uso de tecnologia na empresa, subindo o nível TRL(Technology Readiness Level)
   - **Explicativa:** Com a implementação do robô na fábrica, abririam-se oportunidades para uma maior implementação de tecnologias.
   - **Impacto:** Alto
   - *Justificativa:* O aumento do uso de tecnologia deveria tornar a linha de produção mais eficiente. 
   - **Probabilidade:** 30%
   - *Justificativa:* Há a necessidade de um investimento maior, por parte da empresa, no setor de tecnologia, especialmente em dados, para aumentar o nível e complexidades das soluções desenvolvidas.
  
5. Diminuição no gasto de água, tornando a empresa mais sustentável
   - **Explicativa:** Com a localização do tubo sujo através do robô, deve-se diminuir a quantidade de vezes em que tubos são limpos de forma desnecessária. Sendo assim, trazendo uma economia de água para empresa.
  - **Impacto:** Moderado.
  - *Justificativa:* A redução de água é favorável para empresa tanto para a redução de custos, quanto no  ambiental. Porém, como a linha de produção exige um alto gasto de água, o impacto não é tão alto.
  - **Probabilidade:** 70%
  - *Justificativa:* a probabilidade do evento ocorrer é alta, pois o projeto inclui a verificação dos tubos antes da limpeza.
 
6. Realocação de funcionários para tarefas mais adequadas
   - **Explicativa:** A realocação dos funcionários permitiria que a empresa possa ter um aproveito maior do funcionário, explorando melhor suas habilidades.
   - **Impacto:** Moderado
   - *Justificativa:* O impacto é moderado devido ao fato de que não necessariamente existem outras tarefas para serem realizadas. Porém, há a possibilidade de corte de mão de obra, o que traria uma economia de gastos.
   - **Probabilidade:** 50%
   - *Justificativa:* A probabilidade diz respeito ao tipo de solução desenvolvida, visto que possivelmente o mvp não substituirá o funcionário. Porém, uma possível implementação futura poderia realizar isso.