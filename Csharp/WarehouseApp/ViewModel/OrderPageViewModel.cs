using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WarehouseApp.ViewModel
{
    public class OrderPageViewModel
    {
        public ObservableCollection<CartListItemViewModel> Items { get; }

        public OrderPageViewModel()
        {
            Items = new ObservableCollection<CartListItemViewModel>
        {
            new CartListItemViewModel("Прокладки"),
            new CartListItemViewModel("Болты"),
            new CartListItemViewModel("Гайки"),
            new CartListItemViewModel("Бруски"),
            new CartListItemViewModel("Шайбы"),
        };
        }
    }
}
