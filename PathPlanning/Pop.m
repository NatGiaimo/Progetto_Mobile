function [structure,element]= Pop(structure)
%rimuove l'elemento in testa alla struttura
element=structure(1,:);
tmp=structure(2:end,:);
structure=tmp;
end

