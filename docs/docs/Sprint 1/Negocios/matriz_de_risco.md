# Matriz de Riscos

Uma matriz de risco é importante em projetos de tecnologia, pois oferece uma visão holística e estruturada das potenciais ameaças e oportunidades que podem impactar o planejamento e a execução de uma solução. Ao identificar e classificar os riscos e as oportunidades, desde os mais prováveis até os mais catastróficos, a equipe de projeto pode antecipar problemas, desenvolver estratégias de mitigação e estabelecer planos de contingência eficazes. Permitindo uma gestão proativa do projeto, garantindo que os recursos sejam alocados adequadamente, os prazos cumpridos e os objetivos alcançados dentro das expectativas. Além disso, a matriz de risco promove uma comunicação transparente entre os stakeholders, facilitando a tomada de decisões embasadas e a manutenção da confiança no progresso do projeto.

![Matriz_de_Risco](/img/matriz_de_risco.png)
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
    - *Justificativa:* Isso impediria com que o parceiro utilizasse a solução, ou faria com que fosse implementadas em poucas unidades apenas.
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
    - *Justificativa:* Esse risco causaria com que o projeto perdesse o sentido, visto que o principal valor do robô é a identificação de pontos que devem ser limpos pelo robô.
    - **Probabilidade:** 30%
    - *Justificativa:* A probabilidade de 30% é devido a não existência da base de dados, o que trará mais dificuldade durante o projeto para a implementação de visão computacional.

7. A solução não permite clareza na identificação do tubo que deve ser limpo.
    - **Explicativa:** Uma vez que o robô circulará por baixo, é necessário que envie uma forma de sinal para a persona que está limpando o tubo para indetinficá-lo. 
    - **Impacto:** Muito Alto
    - *Justificativa:* Caso não seja permitir indentificar o tubo, a solução perde seu valor agregado.
    - **Probabilidade:** 50%
    - *Justificativa:* Apesar 
  - 