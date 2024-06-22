---
title: Análise Financeira - Solução final
sidebar_position: 3
---

## Solução Final
A solução final proposta pelo grupo "Rebólins" para a Atvos terá finalidade de implementar, na prática, o conceito teórico demonstrado de forma resumida na prova de conceito.

Para tanto, é importante ressaltar que dificilmente será encontrada uma solução comercializada já pronta no mercado e que esteja disponível para compra e uso imediato. Isso faz necessária uma equipe de profissionais capacitados a não apenas idealizar, prototipar e produzir o robô final considerando o ambiente em que precisará se mover dentro do refervedor, mas também a integrar esse robô com um algoritmo de visão computacional para identificar os canos com impurezas e deixar essa informação disponível para um analista na outra ponta.

Nesse sentido, por não fazer parte do objetivo da análise financeira da solução final, especificações de peças do robô, por exemplo, design do chassi, motores, rodas/esteiras, sensores, bateria não serão dadas, embora seu valor seja estimado para o cálculo de precificação.

Assim, o que compete à análise financeira da solução final é examinar: (1) o custo do **salário da equipe** de profissionais que trabalharão desenvolvendo o robô, considerando a estimativa do custo do hardware; (2) a incidência dos **impostos** sobre o projeto e (3) o **lucro** almejado pela equipe.

### Salário da equipe
Visando o sucesso do projeto, é indispensável que profissionais com diferentes capacitações trabalhem juntos. Isso é particularmente importante para a solução da equipe "Rebólins", que envolve um robô, porque segundo o roboticista Josh[1], da Boston Dynamics, para construir robôs modernos é necessária uma equipe multidisciplinar que contenha engenheiros elétricos, engenheiros mecânicos, engenheiros de controle e automação e engenheiros da computação.

Dessa maneira, para tornar a solução final algo tangível, sugere-se uma equipe de oito pessoas que trabalharão no projeto de automatização da inspeção e limpeza de refervedores da Atvos por um período de doze meses. Esses integrantes estarão distribuídos da seguinte forma: 1 product owner, 2 engenheiros elétricos, 2 engenheiros mecânicos, 2 engenheiros da computação e 1 engenheiro de automação e controle.

A partir de uma pesquisa no Glassdoor[2], maior site de vagas e recrutamento do mundo, é possível chegar a médias salariais dos cargos dos integrantes. As seguintes médias foram consideradas:

|Cargo|Média salarial mensal(BRL)|
|-----|--------------------------|
|Product Owner|9100|
|Engenheiro elétrico|9875|
|Engenheiro mecânico|10051|
|Engenheiro da computação|9674|
|Engenheiro de controle e automação|9667|

Para a quantidade de pessoas de cada área e pelo período de um ano, o cálculo fica como o seguinte:
```
Custo(mão de obra) = 9100*1*12 + 9875*2*12 + 10051*2*12 + 9674*2*12 + 9667*1*12
Custo(mão de obra) = 935604
```

> **Nota:**
>
> _apesar de não ser o objetivo da análise financeira da solução final especificar as peças do robô, uma estimativa de custo com materiais de R$150,000.00(cento e cinquenta mil reais) será levada em consideração_

Então:
```
Custo(mão de obra + materiais) = 935604 + 150000
Custo(mão de obra + materiais) = 1085604
```


### Incidência de impostos

A Classificação Nacional de Atividades Econômicas é um código utilizado para identificar quais são as atividades econômicas exercidas por uma empresa[3]. Ela existe para melhorar a fiscalização do governo e está diretamente ligada à alíquota de impostos que incidem sobre determinado negócio. Por exemplo, o projeto da solução proposta pode se enquadrar no CNAE 2869-1/00(Fabricação de máquinas e equipamentos para uso industrial específico não especificados anteriormente, peças e acessórios), fazendo o negócio sujeito ao imposto IPI(Imposto sobre Produtos Industrializados). Outros impostos, como o ICMS(Imposto sobre Circulação de Mercadorias e Serviços), também poderiam ter suas alíquotas modificadas em razão do CNAE 2869-1/00.

No entanto, esta análise financeira não entrará no mérito de separar os impostos conforme o CNAE que se aplicaria ao projeto, no caso, o 2869-1/00. Em relação a isso, será considerado sobre o custo um imposto único, cujo valor será de 17%.

Então:
```
Custo(mão de obra + materiais + imposto) = 1085604/(1 - 0.17)
Custo(mão de obra + materiais + imposto) = 1307956.62
```

### Lucro
O lucro é também um importante aspecto a ser considerado na análise financeira do projeto. É importante calcular o que se fatura após os descontos, inclusive os tributos[4]. A margem de lucratividade planejada para esse projeto é de 20%.

Então:
```
Custo(mão de obra + materiais + imposto + lucro) = 1307956.62*1.20
Custo(mão de obra + materiais + imposto + lucro) = 1569547.94
```

### Sugestão de como diminuir o custo
Para tentar reduzir o custo do projeto, até aqui calculado em R$1,569,547.94, é possível diminuir o período de contratação de alguns profissionais. Isso pode ser feito por meio da divisão de períodos determinados para construir partes específicas do robô. Por exemplo, considerando que um engenheiro mecânico consiga idealizar, prototipar e produzir todas as dependências mecânicas do robô em seis meses, não seria necessário que ele estivesse contratado durante os seis meses restantes. A mesma coisa vale para os engenheiros elétricos e da computação.

No entanto, vale ressaltar que, caso assim fosse feito, não seria possível, por exemplo, ajustar a mecânica do robô no período dos últimos meses. Da mesma forma, se considerado que os engenheiros elétricos pudessem concluir a parte elétrica do robô em três meses, o que os dispensaria do projeto pelos meses restantes a fim de diminuir custo, então o trabalho realizado durante os outros meses teriam de ser feitos com base nessa parte elétrica específica, sem qualquer chance de ela se adaptar a eventuais necessidades.

Nesse sentido, ainda que não seja a melhor opção a fim de maximizar a chance de o projeto dar certo, dado que ele não seria desenvolvido de maneira holística desde seu início até o fim, a sugestão de mudança de tempo de contratação é como o seguinte: 6 meses iniciais para as dependências mecânicas do robô; 3 meses seguintes para as dependências elétricas e 3 meses finais para as dependências de desenvolvimento de software e integração. Então, o custo se tornaria:

```
Custo(mão de obra) = 9100*1*12 + 9875*2*3 + 10051*2*6 + 9674*2*3 + 9667*1*6 = 405108

Custo(mão de obra + materiais) = 405108 + 150000 = 555108

Custo(mão de obra + materiais + imposto) = 555108/(1 - 0.17) = 668804.81

Custo(mão de obra + materiais + imposto + lucro) = 668804.81*1.20 = 802565.77
```

Por fim, o custo passaria de R$1,569,547.94 para um custo mínimo de R$802,565.77. Isso representa uma redução de R$766,982.17.

## Bibliografia
[1]: Boston Dynamics. Ask a Roboticist: Q&A with Josh, a Mechanical Engineer at Boston Dynamics. Disponível em: https://bostondynamics.com/blog/ask-a-roboticist-qa-with-josh-a-mechanical-engineer-at-boston-dynamics/. Acesso em: 21 maio 2024.

[2]: Glassdoor. Salários. Disponível em: https://www.glassdoor.com.br/Sal%C3%A1rios/index.htm. Acesso em: 21 maio 2024.

[3]: CONTABILIZEI. O que é CNAE? Disponível em: https://www.contabilizei.com.br/contabilidade-online/o-que-e-cnae/. Acesso em: 22 maio 2024.

[4]: IBGEM. Lucro: entenda o conceito. Disponível em: https://ibgem.com.br/2022/11/30/lucro-entenda-o-conceito/. Acesso em: 22 maio 2024.