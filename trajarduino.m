% ISSO AQUI VAI FAZER O VETOR DE TRAJETORIA PARA POR NO ARDUINO.
% SIMOUT ? O VETOR DE ANGULOS DAS TRES JUNTAS. O ANGULO ? EM PULSOS.
% A VARIAVEL DE SAIDA ? A "MATRIZ"

s = 0;
for i=1:1:length(simout)
    if(i == length(simout))
        s = strcat(s,'{',int2str(simout(i,1)),',',int2str(simout(i,2)),',',int2str(simout(i,3)),'}');

    else
        s = strcat(s,'{',int2str(simout(i,1)),',',int2str(simout(i,2)),',',int2str(simout(i,3)),'}',',');

    end
end
s = strcat('{',s,'}');
dimensao = strcat('tamanho: ',int2str(length(simout)))
matriz = strcat('mattraj[',int2str(length(simout)),'][2] = ',s,';')

