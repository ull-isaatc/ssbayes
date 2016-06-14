% Función usada para representar los objetivos en la escena

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function  Representar_obj( objetivos )


    hold on
    
    for i=1:size(objetivos,2)
        plot(objetivos(1,i),objetivos(2,i),'m^','Linewidth',2);
        hold on;
    end
    
end

